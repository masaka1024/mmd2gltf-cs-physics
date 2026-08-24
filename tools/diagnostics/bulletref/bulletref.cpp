// ===========================================================================
//  bulletref -- Task 20: run the SAME minimal net in real Bullet 2.75.
//
//  Reads net.txt (produced by `restosc NETDUMP=1`) and rebuilds the net with
//  genuine Bullet 2.75 objects: btRigidBody + btGeneric6DofSpringConstraint +
//  btDiscreteDynamicsWorld + btSequentialImpulseConstraintSolver.
//
//  Nothing in bullet-2.75/src is modified. Row-level observation is done by
//  subclassing btDiscreteDynamicsWorld and overriding solveConstraints(), which
//  is exactly the point where the solver itself calls getInfo1/getInfo2. The
//  same call with the same state is deterministic, so the numbers printed here
//  are the numbers the solver uses.
//
//  Outputs (same schema as the engine side, so they diff directly):
//    net_bullet_state.csv   frame,sub,body,name,px..pz,qx..qw,vx..vz,wx..wz
//    net_bullet_rows.csv    frame,sub,joint,dof,angular,err,targetVel,relVel
//    bullet_stage1.txt      initial-state report (mass props / frames / limits)
//    bullet_stage2_rows.csv full row detail for the first substep
//
//  Sign convention: the CSVs are written in the ENGINE's convention so the two
//  sides compare directly. Bullet's own row is sign-flipped relative to ours:
//    linear  : err_eng = -err_bt, relVel_eng = -relVel_bt, target_eng = -cErr_bt
//    angular : err_eng = -err_bt, relVel_eng = -relVel_bt, target_eng = +cErr_bt
//  bullet_stage2_rows.csv keeps Bullet's raw values as well.
//
//  Usage:
//    bulletref.exe [--net net.txt] [--frames 600] [--substeps 2] [--iters 10]
//                  [--rowframes 20] [--out .] [--erp 0.5] [--nocontact]
//
//    --erp  overrides BOTH the linear and angular 6DOF limit ERP.
//           Bullet 2.75 defaults: angular m_ERP = 0.5, linear (reused from
//           btTranslationalLimitMotor::m_restitution) = 0.5.
//           Our engine uses Joint.Beta = 0.2 for both. Pass --erp 0.2 to take
//           that difference out of the comparison.
// ===========================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"

// ---------------------------------------------------------------------------
//  net.txt parsing
// ---------------------------------------------------------------------------

struct BodySpec {
    int idx = 0;
    std::string name;
    int mode = 1;              // 0 = bone-follow (kinematic), 1 = dynamic
    double mass = 0, invMass = 0;
    int shape = 2;             // 0 sphere, 1 box, 2 capsule
    double size[3] = {0, 0, 0};
    double margin = 0;
    double inertia[3] = {0, 0, 0};
    double pos[3] = {0, 0, 0};
    double quat[4] = {0, 0, 0, 1};
    double linvel[3] = {0, 0, 0};
    double angvel[3] = {0, 0, 0};
    double lindamp = 0, angdamp = 0, friction = 0, restitution = 0;
    int group = 0, mask = 0;
};

struct JointSpec {
    int idx = 0;
    std::string name;
    int type = 0, a = 0, b = 0;
    double faPos[3] = {0, 0, 0}, faQuat[4] = {0, 0, 0, 1};
    double fbPos[3] = {0, 0, 0}, fbQuat[4] = {0, 0, 0, 1};
    double linLo[3] = {0, 0, 0}, linHi[3] = {0, 0, 0};
    double angLo[3] = {0, 0, 0}, angHi[3] = {0, 0, 0};
    double spLin[3] = {0, 0, 0}, spAng[3] = {0, 0, 0};
    double spDamp = 0.1, beta = 0.2;
    int cross = 0;
};

struct NetSpec {
    double gravity[3] = {0, -98, 0};
    double dt = 1.0 / 60.0;
    int substeps = 2, iters = 10;
    double contactBaumgarte = 0.2, slop = 0.005;
    int jsplit = 0, split = 0, jwarm = 0, jwarmang = 0, leverMode = 0, mixedAxes = 0;
    double maxCorrVel = 10;
    std::vector<BodySpec> bodies;
    std::vector<JointSpec> joints;
};

// key=value where the value may span several whitespace-separated numbers.
typedef std::map<std::string, std::vector<std::string> > KV;

static KV parseKV(const std::vector<std::string>& tok, size_t from)
{
    KV kv;
    std::string cur;
    for (size_t i = from; i < tok.size(); ++i) {
        size_t eq = tok[i].find('=');
        if (eq != std::string::npos) {
            cur = tok[i].substr(0, eq);
            kv[cur].clear();
            std::string rest = tok[i].substr(eq + 1);
            if (!rest.empty()) kv[cur].push_back(rest);
        } else if (!cur.empty()) {
            kv[cur].push_back(tok[i]);
        }
    }
    return kv;
}

static double num(const KV& kv, const char* k, int i, double dflt)
{
    KV::const_iterator it = kv.find(k);
    if (it == kv.end() || (int)it->second.size() <= i) return dflt;
    return atof(it->second[i].c_str());
}
static void num3(const KV& kv, const char* k, double* out)
{
    for (int i = 0; i < 3; ++i) out[i] = num(kv, k, i, out[i]);
}
static void num4(const KV& kv, const char* k, double* out)
{
    for (int i = 0; i < 4; ++i) out[i] = num(kv, k, i, out[i]);
}
static std::string str(const KV& kv, const char* k, const char* dflt)
{
    KV::const_iterator it = kv.find(k);
    if (it == kv.end() || it->second.empty()) return dflt;
    return it->second[0];
}

static bool loadNet(const char* path, NetSpec& n)
{
    std::ifstream f(path);
    if (!f) { fprintf(stderr, "[bulletref] cannot open %s\n", path); return false; }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::vector<std::string> tok;
        std::string t;
        while (is >> t) tok.push_back(t);
        if (tok.empty()) continue;

        if (tok[0] == "world") {
            // "world <subkey> <values...>" -- several forms, handled positionally.
            if (tok[1] == "gravity" && tok.size() >= 5) {
                for (int i = 0; i < 3; ++i) n.gravity[i] = atof(tok[2 + i].c_str());
            } else if (tok[1] == "dt") {
                n.dt = atof(tok[2].c_str());
                for (size_t i = 3; i + 1 < tok.size(); i += 2) {
                    if (tok[i] == "substeps") n.substeps = atoi(tok[i + 1].c_str());
                    if (tok[i] == "iters")    n.iters = atoi(tok[i + 1].c_str());
                }
            } else {
                for (size_t i = 1; i + 1 < tok.size(); i += 2) {
                    const std::string& k = tok[i];
                    double v = atof(tok[i + 1].c_str());
                    if (k == "contactBaumgarte") n.contactBaumgarte = v;
                    else if (k == "slop")        n.slop = v;
                    else if (k == "jsplit")      n.jsplit = (int)v;
                    else if (k == "split")       n.split = (int)v;
                    else if (k == "jwarm")       n.jwarm = (int)v;
                    else if (k == "jwarmang")    n.jwarmang = (int)v;
                    else if (k == "leverMode")   n.leverMode = (int)v;
                    else if (k == "mixedAxes")   n.mixedAxes = (int)v;
                    else if (k == "maxCorrVel")  n.maxCorrVel = v;
                }
            }
        } else if (tok[0] == "body") {
            BodySpec b;
            b.idx = atoi(tok[1].c_str());
            KV kv = parseKV(tok, 2);
            b.name = str(kv, "name", "?");
            b.mode = (int)num(kv, "mode", 0, 1);
            b.mass = num(kv, "mass", 0, 0);
            b.invMass = num(kv, "invMass", 0, 0);
            b.shape = (int)num(kv, "shape", 0, 2);
            num3(kv, "size", b.size);
            b.margin = num(kv, "margin", 0, 0);
            num3(kv, "inertia", b.inertia);
            num3(kv, "pos", b.pos);
            num4(kv, "quat", b.quat);
            num3(kv, "linvel", b.linvel);
            num3(kv, "angvel", b.angvel);
            b.lindamp = num(kv, "lindamp", 0, 0);
            b.angdamp = num(kv, "angdamp", 0, 0);
            b.friction = num(kv, "friction", 0, 0);
            b.restitution = num(kv, "restitution", 0, 0);
            b.group = (int)num(kv, "group", 0, 0);
            b.mask = (int)num(kv, "mask", 0, 0);
            n.bodies.push_back(b);
        } else if (tok[0] == "joint") {
            JointSpec j;
            j.idx = atoi(tok[1].c_str());
            KV kv = parseKV(tok, 2);
            j.name = str(kv, "name", "?");
            j.type = (int)num(kv, "type", 0, 0);
            j.a = (int)num(kv, "a", 0, 0);
            j.b = (int)num(kv, "b", 0, 0);
            num3(kv, "faPos", j.faPos);  num4(kv, "faQuat", j.faQuat);
            num3(kv, "fbPos", j.fbPos);  num4(kv, "fbQuat", j.fbQuat);
            num3(kv, "linLo", j.linLo);  num3(kv, "linHi", j.linHi);
            num3(kv, "angLo", j.angLo);  num3(kv, "angHi", j.angHi);
            num3(kv, "spLin", j.spLin);  num3(kv, "spAng", j.spAng);
            j.spDamp = num(kv, "spDamp", 0, 0.1);
            j.beta = num(kv, "beta", 0, 0.2);
            j.cross = (int)num(kv, "cross", 0, 0);
            n.joints.push_back(j);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
//  Task 21: per-iteration row trace.
//
//  btSequentialImpulseConstraintSolver::solveGroup() is virtual, and its body is
//  just  setup() -> iterations() -> writeback.  solveGroupCacheFriendlyIterations()
//  holds NO per-call state outside the (member) pools -- the writeback and the
//  pool clearing live in solveGroup(), not in it -- so calling it N times with
//  m_numIterations = 1 is exactly the same as calling it once with N.
//  (Verified: SOLVER_RANDMIZE_ORDER is off in the default solverMode 0x104, so
//   the `(iteration & 7) == 0` reshuffle never fires either way. --verify checks
//   bit-identity of the whole run against the undecomposed path.)
//
//  That gives the accumulated impulse of every row after every iteration, which
//  is row-local: m_appliedImpulse for row j is only written when row j is solved.
// ---------------------------------------------------------------------------

// タスク47: 接触行の反復単位トレース。ジョイント行の IterRow と同じ流儀。
//   m_tmpSolverContactConstraintPool は法線行、
//   m_tmpSolverContactFrictionConstraintPool は摩擦行 (法線1本につき最大2本)。
//   摩擦行から法線行へは m_frictionIndex で辿れる。
struct ContactIterRow {
    int frame, substep, iter;
    int idxA, idxB;      // net.txt の body 番号
    int pt;              // 法線行のプール内 index (接触点の識別子)
    double ni, t1, t2;   // 累積: 法線 / 摩擦1 / 摩擦2
    int nClamp, tClamp;  // 法線が下限0に張り付いたか / 摩擦が上限に張り付いたか
    double relN;         // 参考: 法線行の m_appliedPushImpulse は使わない
    double rhs, jacInv;  // ★タスク47手順4: 設定値。rhs=目標速度, jacInv=1/(J M^-1 J^T)
    // ★タスク76: 摩擦行そのものの設定値。方向 (world) / 実効質量 / 上下限 / rhs。
    //   「同じ mu でも当エンジンの方がよく滑る」を行レベルで見るのに要る。
    double t1dir[3], t1Jac, t1Rhs, t1Lo, t1Hi, t1Fric;
    double nDir[3];      // 法線行の m_contactNormal (向きの基準合わせ用)
};

struct IterRow {
    int frame, substep, iter;
    int cons;            // caller-side joint index (net.txt order)
    int solverIdx;       // index into the constraint array passed to solveGroup
    int dof, angular;
    double axis[3];
    double errBt, posErrBt, relVelSetupBt;
    double jac, lower, upper;
    double applied, dApplied;
    int clamped;
};

class RefSolver : public btSequentialImpulseConstraintSolver
{
public:
    RefSolver() : capture(false), frame(0), substep(0) {}

    bool capture;
    int frame, substep;
    std::vector<IterRow> rows;
    std::vector<ContactIterRow> crows;
    // btCollisionObject* -> net.txt の body 番号 (main が埋める)
    std::map<const btCollisionObject*, int> bodyOf;
    // constraint pointer -> caller-side joint index (filled by main)
    std::map<const btTypedConstraint*, int> jointOf;

    virtual btScalar solveGroup(btCollisionObject** bodies, int numBodies,
                                btPersistentManifold** manifoldPtr, int numManifolds,
                                btTypedConstraint** constraints, int numConstraints,
                                const btContactSolverInfo& infoGlobal,
                                btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,
                                btDispatcher* dispatcher)
    {
        if (!capture)
            return btSequentialImpulseConstraintSolver::solveGroup(
                bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints,
                infoGlobal, debugDrawer, stackAlloc, dispatcher);

        solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, numManifolds,
                                     constraints, numConstraints, infoGlobal, debugDrawer, stackAlloc);

        // ---- identify every non-contact row: (constraint, dof, angular) ----
        // The pool is filled constraint by constraint, rows in the order
        // getInfo2 produces them: active linear dof 0,1,2 then active angular 0,1,2.
        std::vector<std::pair<int, int> > id;      // (dof, angular) per pool row
        std::vector<int> consOf;
        for (int i = 0; i < numConstraints; ++i) {
            btGeneric6DofConstraint* d6 = dynamic_cast<btGeneric6DofConstraint*>(constraints[i]);
            if (!d6) continue;
            for (int k = 0; k < 3; ++k)
                if (d6->getTranslationalLimitMotor()->needApplyForce(k)) {
                    id.push_back(std::make_pair(k, 0)); consOf.push_back(i);
                }
            for (int k = 0; k < 3; ++k)
                if (d6->getRotationalLimitMotor(k)->needApplyTorques()) {
                    id.push_back(std::make_pair(k, 1)); consOf.push_back(i);
                }
            // ★btDiscreteDynamicsWorld::solveConstraints sorts the constraint array by
            //   island, so the index into `constraints` is NOT the caller's joint index.
            //   jointOf maps the pointer back to the order in net.txt.
        }
        const int n = m_tmpSolverNonContactConstraintPool.size();
        if ((int)id.size() != n) {
            fprintf(stderr, "[bulletref] row identification mismatch: derived %d, pool %d\n",
                    (int)id.size(), n);
            id.clear(); consOf.clear();
            for (int j = 0; j < n; ++j) { id.push_back(std::make_pair(-1, -1)); consOf.push_back(-1); }
        }

        // ---- setup-time values (deltas are zero here, so rel_vel is the real one) ----
        std::vector<IterRow> base(n);
        for (int j = 0; j < n; ++j) {
            const btSolverConstraint& c = m_tmpSolverNonContactConstraintPool[j];
            // Take the bodies from the constraint, not from the solver-body pool:
            // static/kinematic objects share a fixed solver body whose
            // m_originalBody is null, so dereferencing the pool crashes.
            btRigidBody* rbA = 0; btRigidBody* rbB = 0;
            if (consOf[j] >= 0) {
                rbA = &constraints[consOf[j]]->getRigidBodyA();
                rbB = &constraints[consOf[j]]->getRigidBodyB();
            }
            btVector3 vA(0, 0, 0), wA(0, 0, 0), vB(0, 0, 0), wB(0, 0, 0);
            if (rbA) { vA = rbA->getLinearVelocity(); wA = rbA->getAngularVelocity(); }
            if (rbB) { vB = rbB->getLinearVelocity(); wB = rbB->getAngularVelocity(); }
            btScalar relVel = c.m_contactNormal.dot(vA) + c.m_relpos1CrossNormal.dot(wA)
                            - c.m_contactNormal.dot(vB) + c.m_relpos2CrossNormal.dot(wB);
            IterRow r;
            r.frame = frame; r.substep = substep; r.iter = -1;
            r.cons = -1;
            r.solverIdx = consOf[j];
            if (consOf[j] >= 0) {
                std::map<const btTypedConstraint*, int>::const_iterator it =
                    jointOf.find(constraints[consOf[j]]);
                r.cons = (it != jointOf.end()) ? it->second : -1;
            }
            r.dof = id[j].first; r.angular = id[j].second;
            const btVector3& ax = r.angular ? c.m_relpos1CrossNormal : c.m_contactNormal;
            for (int k = 0; k < 3; ++k) r.axis[k] = ax[k];
            r.jac = c.m_jacDiagABInv;
            r.lower = c.m_lowerLimit; r.upper = c.m_upperLimit;
            r.relVelSetupBt = relVel;
            // m_rhs = (positionalError - rel_vel) * jacDiagABInv  ->  recover positionalError
            r.posErrBt = (r.jac != 0.0) ? (c.m_rhs / r.jac + relVel) : 0.0;
            r.errBt = 0.0;
            if (r.solverIdx >= 0 && r.dof >= 0) {
                btGeneric6DofConstraint* d6 = dynamic_cast<btGeneric6DofConstraint*>(constraints[r.solverIdx]);
                r.errBt = r.angular ? d6->getRotationalLimitMotor(r.dof)->m_currentLimitError
                                    : d6->getTranslationalLimitMotor()->m_currentLimitError[r.dof];
            }
            r.applied = 0.0; r.dApplied = 0.0; r.clamped = 0;
            base[j] = r;
            rows.push_back(r);
        }

        // ★タスク47: 接触点ポインタ -> (剛体A, 剛体B) の索引。
        //   btSolverConstraint は solver body を **index** で持つが、静的/キネマティック剛体は
        //   固定の solver body を共有し m_originalBody が null なので、そこからは剛体を引けない。
        //   代わりに m_originalContactPoint (= &btManifoldPoint) をキーにする。これは一意。
        std::map<const void*, std::pair<const btCollisionObject*, const btCollisionObject*> > ptOf;
        for (int mi = 0; mi < numManifolds; ++mi) {
            const btPersistentManifold* mf = manifoldPtr[mi];
            const btCollisionObject* ba = (const btCollisionObject*)mf->getBody0();
            const btCollisionObject* bb = (const btCollisionObject*)mf->getBody1();
            for (int k = 0; k < mf->getNumContacts(); ++k)
                ptOf[(const void*)&mf->getContactPoint(k)] = std::make_pair(ba, bb);
        }

        // ---- iterate one at a time ----
        btContactSolverInfo one = infoGlobal;
        one.m_numIterations = 1;
        std::vector<double> prev(n, 0.0);
        for (int it = 0; it < infoGlobal.m_numIterations; ++it) {
            solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr, numManifolds,
                                              constraints, numConstraints, one, debugDrawer, stackAlloc);
            for (int j = 0; j < n; ++j) {
                const btSolverConstraint& c = m_tmpSolverNonContactConstraintPool[j];
                IterRow r = base[j];
                r.iter = it;
                r.applied = c.m_appliedImpulse;
                r.dApplied = r.applied - prev[j];
                r.clamped = (r.applied == r.lower || r.applied == r.upper) ? 1 : 0;
                prev[j] = r.applied;
                rows.push_back(r);
            }
            // ---- 接触行 (タスク47) ----
            int nc = m_tmpSolverContactConstraintPool.size();
            int nf = m_tmpSolverContactFrictionConstraintPool.size();
            std::vector<double> ft1(nc, 0.0), ft2(nc, 0.0);
            std::vector<int>    fc1(nc, 0);
            std::vector<const btSolverConstraint*> fr1(nc, (const btSolverConstraint*)0);
            for (int j = 0; j < nf; ++j) {
                const btSolverConstraint& f = m_tmpSolverContactFrictionConstraintPool[j];
                int ni = f.m_frictionIndex;
                if (ni < 0 || ni >= nc) continue;
                if (ft1[ni] == 0.0 && fc1[ni] == 0) { ft1[ni] = f.m_appliedImpulse; fc1[ni] = 1; fr1[ni] = &f; }
                else                                 ft2[ni] = f.m_appliedImpulse;
            }
            for (int j = 0; j < nc; ++j) {
                const btSolverConstraint& c = m_tmpSolverContactConstraintPool[j];
                ContactIterRow r;
                r.frame = frame; r.substep = substep; r.iter = it;
                const btCollisionObject* oa = 0; const btCollisionObject* ob = 0;
                std::map<const void*, std::pair<const btCollisionObject*, const btCollisionObject*> >
                    ::const_iterator ip = ptOf.find(c.m_originalContactPoint);
                if (ip != ptOf.end()) { oa = ip->second.first; ob = ip->second.second; }
                std::map<const btCollisionObject*, int>::const_iterator ia = bodyOf.find(oa);
                std::map<const btCollisionObject*, int>::const_iterator ib = bodyOf.find(ob);
                r.idxA = (ia == bodyOf.end()) ? -1 : ia->second;
                r.idxB = (ib == bodyOf.end()) ? -1 : ib->second;
                r.pt = j;
                r.ni = c.m_appliedImpulse;
                r.t1 = ft1[j]; r.t2 = ft2[j];
                r.nClamp = (c.m_appliedImpulse <= 0.0) ? 1 : 0;
                r.tClamp = 0;
                r.relN = 0.0;
                r.rhs = c.m_rhs; r.jacInv = c.m_jacDiagABInv;
                for (int a = 0; a < 3; ++a) r.nDir[a] = c.m_contactNormal[a];
                if (fr1[j]) {
                    const btSolverConstraint& f = *fr1[j];
                    for (int a = 0; a < 3; ++a) r.t1dir[a] = f.m_contactNormal[a];
                    r.t1Jac = f.m_jacDiagABInv; r.t1Rhs = f.m_rhs;
                    r.t1Lo = f.m_lowerLimit;    r.t1Hi = f.m_upperLimit;
                    r.t1Fric = f.m_friction;
                    r.tClamp = (f.m_appliedImpulse <= f.m_lowerLimit ||
                                f.m_appliedImpulse >= f.m_upperLimit) ? 1 : 0;
                } else {
                    for (int a = 0; a < 3; ++a) r.t1dir[a] = 0.0;
                    r.t1Jac = r.t1Rhs = r.t1Lo = r.t1Hi = r.t1Fric = 0.0;
                }
                crows.push_back(r);
            }
        }

        // ---- replicate the tail of the base solveGroup() ----
        int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
        for (int j = 0; j < numPoolConstraints; j++) {
            const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[j];
            btManifoldPoint* pt = (btManifoldPoint*)solveManifold.m_originalContactPoint;
            pt->m_appliedImpulse = solveManifold.m_appliedImpulse;
            if (infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING) {
                pt->m_appliedImpulseLateral1 =
                    m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
                pt->m_appliedImpulseLateral2 =
                    m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex + 1].m_appliedImpulse;
            }
        }
        if (infoGlobal.m_splitImpulse) {
            for (int i = 0; i < m_tmpSolverBodyPool.size(); i++)
                m_tmpSolverBodyPool[i].writebackVelocity(infoGlobal.m_timeStep);
        } else {
            for (int i = 0; i < m_tmpSolverBodyPool.size(); i++)
                m_tmpSolverBodyPool[i].writebackVelocity();
        }
        m_tmpSolverBodyPool.resize(0);
        m_tmpSolverContactConstraintPool.resize(0);
        m_tmpSolverNonContactConstraintPool.resize(0);
        m_tmpSolverContactFrictionConstraintPool.resize(0);
        return 0.f;
    }
};

// ---------------------------------------------------------------------------
//  Row observation: subclass the world and look at the constraints exactly
//  where the solver sets them up.
// ---------------------------------------------------------------------------

struct RowSample {
    int joint;
    int dof;
    int angular;
    // Bullet raw
    double errBt;          // limot->m_currentLimitError
    double cErrBt;         // info->m_constraintError  (= +/- k*err)
    double relVelBt;       // J1.v  (Bullet's rel_vel at setup)
    double jacDiagABInv;
    double lower, upper;
    double axis[3];
    double j1ang[3], j2ang[3];
    // engine convention
    double errEng, targetEng, relVelEng;
};

class RefWorld : public btDiscreteDynamicsWorld
{
public:
    RefWorld(btDispatcher* d, btBroadphaseInterface* b,
             btConstraintSolver* s, btCollisionConfiguration* c)
        : btDiscreteDynamicsWorld(d, b, s, c), sampling(false), lastDt(0) {}

    bool sampling;
    double lastDt;
    std::vector<RowSample> rows;
    std::vector<std::string>* jointNames;

protected:
    virtual void solveConstraints(btContactSolverInfo& info)
    {
        if (sampling) { lastDt = info.m_timeStep; sample(info); }
        btDiscreteDynamicsWorld::solveConstraints(info);
    }

private:
    void sample(const btContactSolverInfo& info)
    {
        rows.clear();
        // 6 rows max per 6DOF; the J arrays are indexed by row*rowskip so every
        // array is allocated with the same stride.
        const int ROWSKIP = 4;
        const int MAXROW = 6;
        for (int ci = 0; ci < getNumConstraints(); ++ci) {
            btTypedConstraint* c = getConstraint(ci);
            btGeneric6DofConstraint* d6 = dynamic_cast<btGeneric6DofConstraint*>(c);
            if (!d6) continue;

            btScalar J1lin[MAXROW * ROWSKIP], J1ang[MAXROW * ROWSKIP];
            btScalar J2ang[MAXROW * ROWSKIP];
            btScalar cErr[MAXROW * ROWSKIP], cfm[MAXROW * ROWSKIP];
            btScalar lo[MAXROW * ROWSKIP], hi[MAXROW * ROWSKIP];
            memset(J1lin, 0, sizeof(J1lin)); memset(J1ang, 0, sizeof(J1ang));
            memset(J2ang, 0, sizeof(J2ang)); memset(cErr, 0, sizeof(cErr));
            memset(cfm, 0, sizeof(cfm));
            for (int i = 0; i < MAXROW * ROWSKIP; ++i) { lo[i] = -SIMD_INFINITY; hi[i] = SIMD_INFINITY; }

            btTypedConstraint::btConstraintInfo2 i2;
            i2.fps = btScalar(1.) / info.m_timeStep;
            i2.erp = info.m_erp;
            i2.m_J1linearAxis = J1lin;
            i2.m_J1angularAxis = J1ang;
            i2.m_J2linearAxis = 0;
            i2.m_J2angularAxis = J2ang;
            i2.rowskip = ROWSKIP;
            i2.m_constraintError = cErr;
            i2.cfm = cfm;
            i2.m_lowerLimit = lo;
            i2.m_upperLimit = hi;
            i2.m_numIterations = info.m_numIterations;
            d6->getInfo2(&i2);

            // getInfo2 packs the produced rows contiguously starting at row 0,
            // in the order  linear dof0,1,2 (only those that need force), then
            // angular dof0,1,2. Re-derive which dof each row belongs to the
            // same way btGeneric6DofConstraint does.
            btRigidBody& rbA = d6->getRigidBodyA();
            btRigidBody& rbB = d6->getRigidBodyB();
            std::vector<std::pair<int, int> > order;   // (dof, angular)
            for (int i = 0; i < 3; ++i)
                if (d6->getTranslationalLimitMotor()->needApplyForce(i)) order.push_back(std::make_pair(i, 0));
            for (int i = 0; i < 3; ++i)
                if (d6->getRotationalLimitMotor(i)->needApplyTorques()) order.push_back(std::make_pair(i, 1));

            for (size_t r = 0; r < order.size(); ++r) {
                int srow = (int)r * ROWSKIP;
                RowSample s;
                s.joint = ci;
                s.dof = order[r].first;
                s.angular = order[r].second;
                s.errBt = s.angular
                    ? d6->getRotationalLimitMotor(s.dof)->m_currentLimitError
                    : d6->getTranslationalLimitMotor()->m_currentLimitError[s.dof];
                s.cErrBt = cErr[srow];
                s.lower = lo[srow];
                s.upper = hi[srow];

                btVector3 n(J1lin[srow], J1lin[srow + 1], J1lin[srow + 2]);
                btVector3 a1(J1ang[srow], J1ang[srow + 1], J1ang[srow + 2]);
                btVector3 a2(J2ang[srow], J2ang[srow + 1], J2ang[srow + 2]);
                for (int k = 0; k < 3; ++k) {
                    s.axis[k] = s.angular ? a1[k] : n[k];
                    s.j1ang[k] = a1[k];
                    s.j2ang[k] = a2[k];
                }

                // rel_vel exactly as btSequentialImpulseConstraintSolver computes it
                btScalar v1 = n.dot(rbA.getLinearVelocity()) + a1.dot(rbA.getAngularVelocity());
                btScalar v2 = -n.dot(rbB.getLinearVelocity()) + a2.dot(rbB.getAngularVelocity());
                s.relVelBt = v1 + v2;

                // jacDiagABInv, same expression as the solver's
                btVector3 iMJlA = n * rbA.getInvMass();
                btVector3 iMJaA = rbA.getInvInertiaTensorWorld() * a1;
                btVector3 iMJlB = n * rbB.getInvMass();
                btVector3 iMJaB = rbB.getInvInertiaTensorWorld() * a2;
                btScalar sum = iMJlA.dot(n) + iMJaA.dot(a1) + iMJlB.dot(n) + iMJaB.dot(a2);
                s.jacDiagABInv = sum != 0 ? 1.0 / sum : 0.0;

                s.errEng = -s.errBt;
                s.relVelEng = -s.relVelBt;
                s.targetEng = s.angular ? s.cErrBt : -s.cErrBt;
                rows.push_back(s);
            }
        }
    }
};

// ---------------------------------------------------------------------------

static const char* argStr(int argc, char** argv, const char* k, const char* d)
{
    for (int i = 1; i + 1 < argc; ++i) if (!strcmp(argv[i], k)) return argv[i + 1];
    return d;
}
static int argInt(int argc, char** argv, const char* k, int d)
{
    const char* s = argStr(argc, argv, k, 0);
    return s ? atoi(s) : d;
}
static double argDbl(int argc, char** argv, const char* k, double d)
{
    const char* s = argStr(argc, argv, k, 0);
    return s ? atof(s) : d;
}
static bool argFlag(int argc, char** argv, const char* k)
{
    for (int i = 1; i < argc; ++i) if (!strcmp(argv[i], k)) return true;
    return false;
}

int main(int argc, char** argv)
{
    const char* netPath = argStr(argc, argv, "--net", "net.txt");
    std::string outDir = argStr(argc, argv, "--out", ".");
    int frames = argInt(argc, argv, "--frames", 600);
    int rowFrames = argInt(argc, argv, "--rowframes", 20);
    bool noContact = argFlag(argc, argv, "--nocontact");
    bool rowTrace = argFlag(argc, argv, "--rowtrace");
    // ★タスク50: 接触だけ全フレーム出す。rowtrace は 3600F だと数GBになるので分離した。
    bool allContacts = argFlag(argc, argv, "--allcontacts");
    bool keepMargin = argFlag(argc, argv, "--keepmargin");  // do NOT override with the engine margin
    int dumpStateAt = argInt(argc, argv, "--dumpstate", -1);
    const char* initState = argStr(argc, argv, "--initstate", 0);

    NetSpec net;
    if (!loadNet(netPath, net)) return 1;
    int substeps = argInt(argc, argv, "--substeps", net.substeps);
    int iters = argInt(argc, argv, "--iters", net.iters);
    // -1 means "leave Bullet's own defaults" (angular 0.5 / linear 0.5).
    double erp = argDbl(argc, argv, "--erp", -1.0);

    printf("bulletref : net=%s  bodies=%d joints=%d  frames=%d substeps=%d iters=%d\n",
           netPath, (int)net.bodies.size(), (int)net.joints.size(), frames, substeps, iters);

    // ─ world ─
    btDefaultCollisionConfiguration* cfg = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* disp = new btCollisionDispatcher(cfg);
    btDbvtBroadphase* broad = new btDbvtBroadphase();
    RefSolver* solver = new RefSolver();
    RefWorld* world = new RefWorld(disp, broad, solver, cfg);
    world->setGravity(btVector3(net.gravity[0], net.gravity[1], net.gravity[2]));
    world->getSolverInfo().m_numIterations = iters;
    world->getSolverInfo().m_splitImpulse = false;

    std::vector<btRigidBody*> bodies(net.bodies.size(), (btRigidBody*)0);
    std::vector<btCollisionShape*> shapes;
    std::vector<double> marginDefault;

    for (size_t i = 0; i < net.bodies.size(); ++i) {
        const BodySpec& s = net.bodies[i];
        btCollisionShape* sh = 0;
        if (s.shape == 0)      sh = new btSphereShape(btScalar(s.size[0]));
        else if (s.shape == 1) sh = new btBoxShape(btVector3(s.size[0], s.size[1], s.size[2]));
        else                   sh = new btCapsuleShape(btScalar(s.size[0]), btScalar(s.size[1]));
        marginDefault.push_back((double)sh->getMargin());
        // The engine's per-shape margin convention differs from Bullet's
        // (engine: sphere/capsule = radius, box = min(halfExtent)*0.04;
        //  Bullet: box = min(0.04, min(halfExtent)*0.1)). Contact depth depends
        // directly on the margin, so use the engine's value and report the delta.
        if (!keepMargin) sh->setMargin(btScalar(s.margin));
        shapes.push_back(sh);

        btTransform tr;
        tr.setRotation(btQuaternion(s.quat[0], s.quat[1], s.quat[2], s.quat[3]));
        tr.setOrigin(btVector3(s.pos[0], s.pos[1], s.pos[2]));

        bool kinematic = (s.mode == 0);
        btScalar mass = kinematic ? btScalar(0) : btScalar(s.mass);
        // ★Use the engine's inertia tensor verbatim instead of Bullet's own
        //   calculateLocalInertia. Any difference in the inertia formula must not
        //   leak into the solver comparison; stage 1 reports the difference.
        btVector3 inertia(s.inertia[0], s.inertia[1], s.inertia[2]);

        btDefaultMotionState* ms = new btDefaultMotionState(tr);
        btRigidBody::btRigidBodyConstructionInfo ci(mass, ms, sh, inertia);
        btRigidBody* rb = new btRigidBody(ci);
        rb->setMassProps(mass, inertia);
        rb->updateInertiaTensor();
        rb->setWorldTransform(tr);
        rb->setDamping(btScalar(s.lindamp), btScalar(s.angdamp));
        rb->setFriction(btScalar(s.friction));
        rb->setRestitution(btScalar(s.restitution));
        rb->setLinearVelocity(btVector3(s.linvel[0], s.linvel[1], s.linvel[2]));
        rb->setAngularVelocity(btVector3(s.angvel[0], s.angvel[1], s.angvel[2]));
        rb->setSleepingThresholds(btScalar(0), btScalar(0));      // never sleep
        solver->bodyOf[rb] = (int)i;   // タスク47: 接触行で net.txt の body 番号を出すため
        rb->setActivationState(DISABLE_DEACTIVATION);
        if (kinematic) {
            rb->setCollisionFlags(rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        }
        // PMX filtering: A collides with B iff (A.mask >> B.group)&1 and (B.mask >> A.group)&1.
        // btDbvtBroadphase only does a single AND pair test, so the mask is passed as-is and
        // the extra half of the test is done by the dispatcher's needsCollision below.
        short group = (short)(1 << s.group);
        short mask = noContact ? (short)0 : (short)s.mask;
        world->addRigidBody(rb, group, mask);
        bodies[i] = rb;
    }

    std::vector<btGeneric6DofSpringConstraint*> cons;
    std::vector<std::string> jointNames;
    for (size_t i = 0; i < net.joints.size(); ++i) {
        const JointSpec& j = net.joints[i];
        btTransform fa, fb;
        fa.setRotation(btQuaternion(j.faQuat[0], j.faQuat[1], j.faQuat[2], j.faQuat[3]));
        fa.setOrigin(btVector3(j.faPos[0], j.faPos[1], j.faPos[2]));
        fb.setRotation(btQuaternion(j.fbQuat[0], j.fbQuat[1], j.fbQuat[2], j.fbQuat[3]));
        fb.setOrigin(btVector3(j.fbPos[0], j.fbPos[1], j.fbPos[2]));

        btGeneric6DofSpringConstraint* c = new btGeneric6DofSpringConstraint(
            *bodies[j.a], *bodies[j.b], fa, fb, true /* useLinearReferenceFrameA */);
        c->setLinearLowerLimit(btVector3(j.linLo[0], j.linLo[1], j.linLo[2]));
        c->setLinearUpperLimit(btVector3(j.linHi[0], j.linHi[1], j.linHi[2]));
        c->setAngularLowerLimit(btVector3(j.angLo[0], j.angLo[1], j.angLo[2]));
        c->setAngularUpperLimit(btVector3(j.angHi[0], j.angHi[1], j.angHi[2]));
        for (int k = 0; k < 3; ++k) {
            // Do NOT call setDamping(): PMX carries no damping value, so the faithful choice is
            // Bullet's own default m_springDamping = 1.0.  The engine's Joint.SpringDamping (0.1)
            // is dead code in the explicit path and must not leak in here -- passing it made the
            // motor target velocity exactly 10x too small on the Bullet side (task 34).
            if (j.spLin[k] != 0) { c->enableSpring(k, true); c->setStiffness(k, btScalar(j.spLin[k])); }
            if (j.spAng[k] != 0) { c->enableSpring(k + 3, true); c->setStiffness(k + 3, btScalar(j.spAng[k])); }
        }
        // Do NOT call setEquilibriumPoint(): it would capture the diff at construction time,
        // while the engine uses ClampToLimit(0, lo, hi) as the equilibrium.  Bullet's ctor
        // default m_equilibriumPoint = 0 matches the engine whenever 0 lies inside the limits.
        // (task 32 -- keep the two spring definitions identical)
        // c->setEquilibriumPoint();
        if (erp >= 0) {
            c->getTranslationalLimitMotor()->m_restitution = btScalar(erp);  // linear ERP (2.75 reuses this field)
            for (int k = 0; k < 3; ++k) c->getRotationalLimitMotor(k)->m_ERP = btScalar(erp);
        }
        // PMX does NOT disable collisions between jointed bodies (it uses the group/mask
        // system instead), so pass false here to stay faithful to the engine.
        world->addConstraint(c, false);
        cons.push_back(c);
        jointNames.push_back(j.name);
        solver->jointOf[c] = (int)i;
    }
    world->jointNames = &jointNames;

    // ---- optional: transplant an exact starting state (task 21) ----
    // Both sides start from the SAME non-degenerate pose, so stage (a) compares
    // row construction on identical input instead of on drifted states.
    if (initState) {
        std::ifstream fst(initState);
        if (!fst) { fprintf(stderr, "[bulletref] cannot open %s\n", initState); return 1; }
        std::string line;
        int nst = 0;
        while (std::getline(fst, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream is(line);
            std::vector<std::string> tok; std::string t;
            while (is >> t) tok.push_back(t);
            if (tok.size() < 2 || tok[0] != "state") continue;
            int bi = atoi(tok[1].c_str());
            if (bi < 0 || bi >= (int)bodies.size()) continue;
            KV kv = parseKV(tok, 2);
            double pos[3] = {0,0,0}, quat[4] = {0,0,0,1}, lv[3] = {0,0,0}, av[3] = {0,0,0};
            num3(kv, "pos", pos); num4(kv, "quat", quat);
            num3(kv, "linvel", lv); num3(kv, "angvel", av);
            btTransform tr;
            tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
            tr.setOrigin(btVector3(pos[0], pos[1], pos[2]));
            bodies[bi]->setWorldTransform(tr);
            bodies[bi]->getMotionState()->setWorldTransform(tr);
            bodies[bi]->setInterpolationWorldTransform(tr);
            bodies[bi]->updateInertiaTensor();
            bodies[bi]->setLinearVelocity(btVector3(lv[0], lv[1], lv[2]));
            bodies[bi]->setAngularVelocity(btVector3(av[0], av[1], av[2]));
            ++nst;
        }
        printf("  initstate: %d bodies from %s\n", nst, initState);
        for (size_t i2 = 0; i2 < cons.size(); ++i2) cons[i2]->calculateTransforms();
    }

    // ─────────────────────────────────────────────────────────────
    //  Stage 1: initial-state report
    // ─────────────────────────────────────────────────────────────
    {
        std::string p = outDir + "/bullet_stage1.txt";
        FILE* f = fopen(p.c_str(), "w");
        fprintf(f, "# bulletref stage1 -- initial state, Bullet 2.75 side\n");
        fprintf(f, "gravity %.9g %.9g %.9g   dt %.9g  substeps %d  iters %d\n",
                (double)world->getGravity().x(), (double)world->getGravity().y(),
                (double)world->getGravity().z(), net.dt, substeps, iters);
        fprintf(f, "solverInfo erp %.9g  erp2 %.9g  splitImpulse %d  warmstartingFactor %.9g  solverMode 0x%x\n",
                (double)world->getSolverInfo().m_erp, (double)world->getSolverInfo().m_erp2,
                (int)world->getSolverInfo().m_splitImpulse,
                (double)world->getSolverInfo().m_warmstartingFactor,
                world->getSolverInfo().m_solverMode);
        fprintf(f, "\n# mass properties: engine inertia vs Bullet's own calculateLocalInertia\n");
        for (size_t i = 0; i < net.bodies.size(); ++i) {
            const BodySpec& s = net.bodies[i];
            btVector3 own(0, 0, 0);
            if (s.mass > 0) shapes[i]->calculateLocalInertia(btScalar(s.mass), own);
            fprintf(f, "body %d %-10s mass %.9g  invMass %.9g\n", (int)i, s.name.c_str(),
                    (double)bodies[i]->getInvMass() > 0 ? 1.0 / bodies[i]->getInvMass() : 0.0,
                    (double)bodies[i]->getInvMass());
            fprintf(f, "   inertia  engine %.9g %.9g %.9g\n", s.inertia[0], s.inertia[1], s.inertia[2]);
            fprintf(f, "            bullet %.9g %.9g %.9g   (ratio %.6g %.6g %.6g)\n",
                    (double)own.x(), (double)own.y(), (double)own.z(),
                    s.inertia[0] != 0 ? own.x() / s.inertia[0] : 0.0,
                    s.inertia[1] != 0 ? own.y() / s.inertia[1] : 0.0,
                    s.inertia[2] != 0 ? own.z() / s.inertia[2] : 0.0);
            fprintf(f, "   margin  engine %.9g  bullet-default %.9g  (ratio %.6g)\n",
                    s.margin, marginDefault[i], marginDefault[i] != 0 ? s.margin / marginDefault[i] : 0.0);
            const btTransform& tr = bodies[i]->getWorldTransform();
            btQuaternion q = tr.getRotation();
            fprintf(f, "   pos %.9g %.9g %.9g  quat %.9g %.9g %.9g %.9g\n",
                    (double)tr.getOrigin().x(), (double)tr.getOrigin().y(), (double)tr.getOrigin().z(),
                    (double)q.x(), (double)q.y(), (double)q.z(), (double)q.w());
        }
        fprintf(f, "\n# joints: world anchors and limit ERP actually in effect\n");
        for (size_t i = 0; i < cons.size(); ++i) {
            cons[i]->calculateTransforms();
            const btTransform& ta = cons[i]->getCalculatedTransformA();
            const btTransform& tb = cons[i]->getCalculatedTransformB();
            fprintf(f, "joint %d %-12s cross=%d\n", (int)i, jointNames[i].c_str(), net.joints[i].cross);
            fprintf(f, "   anchorA %.9g %.9g %.9g\n", (double)ta.getOrigin().x(), (double)ta.getOrigin().y(), (double)ta.getOrigin().z());
            fprintf(f, "   anchorB %.9g %.9g %.9g\n", (double)tb.getOrigin().x(), (double)tb.getOrigin().y(), (double)tb.getOrigin().z());
            btVector3 d = tb.getOrigin() - ta.getOrigin();
            fprintf(f, "   anchor gap %.9g\n", (double)d.length());
            fprintf(f, "   linear  ERP %.9g (engine Beta %.9g)   angular ERP %.9g %.9g %.9g\n",
                    (double)cons[i]->getTranslationalLimitMotor()->m_restitution, net.joints[i].beta,
                    (double)cons[i]->getRotationalLimitMotor(0)->m_ERP,
                    (double)cons[i]->getRotationalLimitMotor(1)->m_ERP,
                    (double)cons[i]->getRotationalLimitMotor(2)->m_ERP);
            for (int k = 0; k < 3; ++k)
                fprintf(f, "   lin dof%d  lo %.9g hi %.9g  currentDiff %.9g  err %.9g\n", k,
                        (double)cons[i]->getTranslationalLimitMotor()->m_lowerLimit[k],
                        (double)cons[i]->getTranslationalLimitMotor()->m_upperLimit[k],
                        (double)cons[i]->getRelativePivotPosition(k),
                        (double)cons[i]->getTranslationalLimitMotor()->m_currentLimitError[k]);
            for (int k = 0; k < 3; ++k)
                fprintf(f, "   ang dof%d  lo %.9g hi %.9g  angle %.9g\n", k,
                        (double)cons[i]->getRotationalLimitMotor(k)->m_loLimit,
                        (double)cons[i]->getRotationalLimitMotor(k)->m_hiLimit,
                        (double)cons[i]->getAngle(k));
        }
        fclose(f);
        printf("  -> %s\n", p.c_str());
    }

    // ─────────────────────────────────────────────────────────────
    //  Run
    // ─────────────────────────────────────────────────────────────
    std::string sp = outDir + "/net_bullet_state.csv";
    std::string rp = outDir + "/net_bullet_rows.csv";
    std::string s2 = outDir + "/bullet_stage2_rows.csv";
    FILE* fs = fopen(sp.c_str(), "w");
    FILE* fr = fopen(rp.c_str(), "w");
    FILE* f2 = fopen(s2.c_str(), "w");
    fprintf(fs, "frame,sub,body,name,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n");
    fprintf(fr, "frame,sub,joint,dof,angular,err,targetVel,relVel\n");
    fprintf(f2, "frame,sub,joint,dof,angular,errBt,cErrBt,relVelBt,jacDiagABInv,lower,upper,"
                "axisX,axisY,axisZ,j1angX,j1angY,j1angZ,j2angX,j2angY,j2angZ\n");

    std::vector<IterRow> rowsAll;
    std::vector<ContactIterRow> crowsAll;   // タスク47
    // ---- task 28: contact manifold dump (stage (a): contact GENERATION) ----
    FILE* fcon = 0;
    if (rowTrace || allContacts) {
        std::string cp2 = outDir + "/contacts_bullet.csv";
        fcon = fopen(cp2.c_str(), "w");
        fprintf(fcon, "frame,substep,manifold,pt,bodyA,bodyB,pAx,pAy,pAz,pBx,pBy,pBz,"
                      "nx,ny,nz,dist,lifeTime,appliedImpulse,appliedLat1,appliedLat2\n");
        printf("  -> %s\n", cp2.c_str());
    }
    // Raw joint state (all 3 angular + all 3 linear DOFs, whether or not a row is
    // raised) so the two sides' Euler extraction can be compared directly.
    FILE* fang = 0;
    if (rowTrace) {
        std::string ap2 = outDir + "/angles_bullet.csv";
        fang = fopen(ap2.c_str(), "w");
        fprintf(fang, "frame,joint,dof,state,cur,err,linCur,linErr,linState\n");
        printf("  -> %s\n", ap2.c_str());
    }
    const btScalar subDt = btScalar(net.dt / substeps);
    int lateFrom = frames - frames / 3;
    long contactSeen = 0;

    for (int f = 0; f < frames; ++f) {
        if (f == dumpStateAt) {
            std::string dp = outDir + "/initstate.txt";
            FILE* fd = fopen(dp.c_str(), "w");
            fprintf(fd, "# bulletref state snapshot at frame %d (taken BEFORE the step)\n", f);
            for (size_t i2 = 0; i2 < bodies.size(); ++i2) {
                const btTransform& tr = bodies[i2]->getWorldTransform();
                btQuaternion q = tr.getRotation();
                const btVector3& v = bodies[i2]->getLinearVelocity();
                const btVector3& w = bodies[i2]->getAngularVelocity();
                fprintf(fd, "state %d name=%s pos=%.9g %.9g %.9g quat=%.9g %.9g %.9g %.9g "
                            "linvel=%.9g %.9g %.9g angvel=%.9g %.9g %.9g\n",
                        (int)i2, net.bodies[i2].name.c_str(),
                        (double)tr.getOrigin().x(), (double)tr.getOrigin().y(), (double)tr.getOrigin().z(),
                        (double)q.x(), (double)q.y(), (double)q.z(), (double)q.w(),
                        (double)v.x(), (double)v.y(), (double)v.z(),
                        (double)w.x(), (double)w.y(), (double)w.z());
            }
            fclose(fd);
            printf("  -> %s  (frame %d)\n", dp.c_str(), f);
        }
        bool wantRows = (f < rowFrames) || (f >= lateFrom);
        for (int s = 0; s < substeps; ++s) {
            world->sampling = wantRows;
            world->rows.clear();
            solver->frame = f; solver->substep = s;
            solver->capture = rowTrace && wantRows;
            world->stepSimulation(subDt, 1, subDt);
            solver->capture = false;
            if (fang && wantRows && s == 0) {
                for (size_t ci = 0; ci < cons.size(); ++ci) {
                    // getInfo1/getInfo2 already ran calculateTransforms this substep,
                    // so the motors hold the values the solver actually used.
                    for (int k = 0; k < 3; ++k) {
                        btRotationalLimitMotor* rm = cons[ci]->getRotationalLimitMotor(k);
                        btTranslationalLimitMotor* tm = cons[ci]->getTranslationalLimitMotor();
                        fprintf(fang, "%d,%s,%d,%d,%.9g,%.9g,%.9g,%.9g,%d\n",
                                f, jointNames[ci].c_str(), k,
                                rm->m_currentLimit, (double)cons[ci]->getAngle(k),
                                (double)rm->m_currentLimitError,
                                (double)cons[ci]->getRelativePivotPosition(k),
                                (double)tm->m_currentLimitError[k],
                                tm->m_currentLimit[k]);
                    }
                }
            }
            if (rowTrace && wantRows) {
                rowsAll.insert(rowsAll.end(), solver->rows.begin(), solver->rows.end());
                solver->rows.clear();
                crowsAll.insert(crowsAll.end(), solver->crows.begin(), solver->crows.end());
                solver->crows.clear();
            }
            world->sampling = false;
            contactSeen += disp->getNumManifolds();
            if (fcon && (allContacts || wantRows)) {
                for (int mi = 0; mi < disp->getNumManifolds(); ++mi) {
                    btPersistentManifold* mf = disp->getManifoldByIndexInternal(mi);
                    const btCollisionObject* o0 = (const btCollisionObject*)mf->getBody0();
                    const btCollisionObject* o1 = (const btCollisionObject*)mf->getBody1();
                    const char* n0 = "?"; const char* n1 = "?";
                    for (size_t bi = 0; bi < bodies.size(); ++bi) {
                        if ((const btCollisionObject*)bodies[bi] == o0) n0 = net.bodies[bi].name.c_str();
                        if ((const btCollisionObject*)bodies[bi] == o1) n1 = net.bodies[bi].name.c_str();
                    }
                    for (int pi = 0; pi < mf->getNumContacts(); ++pi) {
                        const btManifoldPoint& mp = mf->getContactPoint(pi);
                        fprintf(fcon, "%d,%d,%d,%d,%s,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                                      "%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g\n",
                                f, s, mi, pi, n0, n1,
                                (double)mp.getPositionWorldOnA().x(), (double)mp.getPositionWorldOnA().y(),
                                (double)mp.getPositionWorldOnA().z(),
                                (double)mp.getPositionWorldOnB().x(), (double)mp.getPositionWorldOnB().y(),
                                (double)mp.getPositionWorldOnB().z(),
                                (double)mp.m_normalWorldOnB.x(), (double)mp.m_normalWorldOnB.y(),
                                (double)mp.m_normalWorldOnB.z(),
                                (double)mp.getDistance(), mp.getLifeTime(),
                                (double)mp.getAppliedImpulse(),
                                (double)mp.m_appliedImpulseLateral1, (double)mp.m_appliedImpulseLateral2);
                    }
                }
            }
            if (!wantRows) continue;
            for (size_t r = 0; r < world->rows.size(); ++r) {
                const RowSample& x = world->rows[r];
                fprintf(fr, "%d,%d,%s,%d,%d,%.9g,%.9g,%.9g\n", f, s,
                        jointNames[x.joint].c_str(), x.dof, x.angular,
                        x.errEng, x.targetEng, x.relVelEng);
                if (f == 0 && s == 0)
                    fprintf(f2, "%d,%d,%s,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                                "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                            f, s, jointNames[x.joint].c_str(), x.dof, x.angular,
                            x.errBt, x.cErrBt, x.relVelBt, x.jacDiagABInv, x.lower, x.upper,
                            x.axis[0], x.axis[1], x.axis[2],
                            x.j1ang[0], x.j1ang[1], x.j1ang[2],
                            x.j2ang[0], x.j2ang[1], x.j2ang[2]);
            }
        }
        // state at end of frame (sub = -1), same convention as the engine dump
        for (size_t i = 0; i < bodies.size(); ++i) {
            const btTransform& tr = bodies[i]->getWorldTransform();
            btQuaternion q = tr.getRotation();
            const btVector3& v = bodies[i]->getLinearVelocity();
            const btVector3& w = bodies[i]->getAngularVelocity();
            fprintf(fs, "%d,-1,%d,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    f, (int)i, net.bodies[i].name.c_str(),
                    (double)tr.getOrigin().x(), (double)tr.getOrigin().y(), (double)tr.getOrigin().z(),
                    (double)q.x(), (double)q.y(), (double)q.z(), (double)q.w(),
                    (double)v.x(), (double)v.y(), (double)v.z(),
                    (double)w.x(), (double)w.y(), (double)w.z());
        }
    }
    fclose(fs); fclose(fr); fclose(f2);
    if (fang) fclose(fang);
    if (fcon) fclose(fcon);

    // ---- task 47: per-iteration CONTACT row trace ----
    if (rowTrace) {
        std::string cp3 = outDir + "/contactiter_bullet.csv";
        FILE* fc3 = fopen(cp3.c_str(), "w");
        fprintf(fc3, "frame,iter,bodyA,bodyB,pt,ni,t1,t2,nClamp,tClamp,relN,rhs,jacInv,"
                     "t1x,t1y,t1z,t1Jac,t1Rhs,t1Lo,t1Hi,t1Fric,nx,ny,nz\n");
        for (size_t i = 0; i < crowsAll.size(); ++i) {
            const ContactIterRow& r = crowsAll[i];
            const char* na = (r.idxA >= 0 && r.idxA < (int)net.bodies.size()) ? net.bodies[r.idxA].name.c_str() : "?";
            const char* nb = (r.idxB >= 0 && r.idxB < (int)net.bodies.size()) ? net.bodies[r.idxB].name.c_str() : "?";
            fprintf(fc3, "%d,%d,%s,%s,%d,%.9g,%.9g,%.9g,%d,%d,%.9g,%.9g,%.9g,"
                         "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    r.frame, r.iter, na, nb, r.pt, r.ni, r.t1, r.t2, r.nClamp, r.tClamp, r.relN, r.rhs, r.jacInv,
                    r.t1dir[0], r.t1dir[1], r.t1dir[2], r.t1Jac, r.t1Rhs, r.t1Lo, r.t1Hi, r.t1Fric,
                    r.nDir[0], r.nDir[1], r.nDir[2]);
        }
        fclose(fc3);
        printf("  -> %s  (contact rows per iteration)\n", cp3.c_str());
    }

    // ---- task 21: per-iteration row trace in the common schema ----
    if (rowTrace) {
        std::string tp = outDir + "/rowtrace_bullet.csv";
        FILE* ft = fopen(tp.c_str(), "w");
        fprintf(ft, "frame,substep,iter,joint,dof,angular,axisX,axisY,axisZ,err,bias,targetVel,"
                    "lower,upper,effMass,appliedImpulse,dImpulse,clamped,relVelBefore,relVelAfter,"
                    "errBt,posErrBt,relVelSetupBt,appliedBt,lowerBt,upperBt\n");
        // Sign: Bullet's whole row is the mirror of ours (its rel_vel is measured
        // A-relative-to-B, ours B-relative-to-A), so err / target / rel_vel / impulse
        // all flip together and the impulse limits swap AND negate.
        // The raw Bullet values are kept in the trailing *Bt columns so the sign
        // convention can be re-derived from the data instead of trusted.
        for (size_t i = 0; i < rowsAll.size(); ++i) {
            const IterRow& r = rowsAll[i];
            // Linear rows: Bullet's row is the mirror of ours (Bullet measures rel_vel
            //   A-relative-to-B, we measure it B-relative-to-A), so err / target /
            //   rel_vel / impulse all flip and the impulse limits swap AND negate.
            // Angular rows: assumes the engine runs with Joint.BulletAngleConvention=ON.
            //   With it ON the engine negates the angular row axis, so rel_vel / target /
            //   impulse / limits match DIRECTLY; only err flips, because the formulas
            //   differ (ours bound-cur, Bullet's cur-bound).
            //   With it OFF the two sides compute different angles altogether (task 21),
            //   so no sign mapping makes them comparable.
            const double sgn = r.angular ? +1.0 : -1.0;
            double errEng = -r.errBt;
            double tgtEng = sgn * r.posErrBt;
            double appEng = sgn * r.applied, dEng = sgn * r.dApplied;
            double loEng = r.angular ? r.lower : -r.upper;
            double hiEng = r.angular ? r.upper : -r.lower;
            double relBefore, relAfter;
            if (r.iter < 0) { relBefore = sgn * r.relVelSetupBt; relAfter = NAN; }
            else {
                // relVelAfter = relVelBefore + dImpulse / effMass, and an unclamped
                // row lands exactly on the target; recover "before" from the delta.
                relAfter = NAN; relBefore = NAN;
                if (r.jac != 0.0) {
                    relBefore = tgtEng - dEng / r.jac;
                    relAfter = relBefore + dEng / r.jac;
                }
            }
            const char* jn = (r.cons >= 0 && r.cons < (int)jointNames.size())
                           ? jointNames[r.cons].c_str() : "?";
            fprintf(ft, "%d,%d,%d,%s,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                        "%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    r.frame, r.substep, r.iter, jn, r.dof, r.angular,
                    r.axis[0], r.axis[1], r.axis[2],
                    errEng, tgtEng, tgtEng, loEng, hiEng, r.jac,
                    appEng, dEng, r.clamped, relBefore, relAfter,
                    r.errBt, r.posErrBt, r.relVelSetupBt, r.applied, r.lower, r.upper);
        }
        fclose(ft);
        printf("  -> %s  (rows=%d)\n", tp.c_str(), (int)rowsAll.size());
    }
    printf("  -> %s\n  -> %s\n  -> %s\n", sp.c_str(), rp.c_str(), s2.c_str());
    printf("  manifolds seen (summed over substeps): %ld\n", contactSeen);

    // teardown
    for (size_t i = 0; i < cons.size(); ++i) { world->removeConstraint(cons[i]); delete cons[i]; }
    for (size_t i = 0; i < bodies.size(); ++i) {
        world->removeRigidBody(bodies[i]);
        delete bodies[i]->getMotionState();
        delete bodies[i];
    }
    for (size_t i = 0; i < shapes.size(); ++i) delete shapes[i];
    delete world; delete solver; delete broad; delete disp; delete cfg;
    return 0;
}
