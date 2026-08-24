# タスク77: LEVER=2 を本命候補として全ゲートを再取得する (出荷既定 LEVER=1 との A/B)。
# 出荷既定=env 未設定。ここでは LEVER だけを振る。他フラグは一切触らない。
$ROOT = "C:\mytask2\unity-bullet-physics"
$PMX  = "$ROOT\Assets\testdata\IA.pmx"
$stage = $env:STAGE
function Clear-Env {
  foreach ($k in @("CMARGIN","MODELS","MINNET","DRIVEDP","BONEDP","BODIES","REFFLOOR","FRAMES","SPRINGCLAMP",
                   "SPRINGMOTOR","NETDUMP","KEEPBODY","EXTRABODIES","INITSTATE","JOINTORDER","ROWTRACE",
                   "CONTACTCSV","OUTDIR","SMOKE","SLOP","MANSTATS","ITERS","SUBSTEPS","JOINTS_FIRST",
                   "NOCONTACT","ANCHORDUMP","REPLAY","OUT","AB","MIXAXES","CNBF","BAUM","CSET","NORMFIRST",
                   "CPOOL","FRICALIGN","FRICMUL","JBETA","MAXCORR","ALIGN","ALPHA","ORDER2","INERTIAMARGIN",
                   "RUNAWAY","RUNAWAY_BONE","ANGCONV","AXES","CTHRESH","ROTEXP","CRHS","CMAN","LIMGATE",
                   "SYMDIST","LEVER","DPPART","BONECSV_OUT","DPFRAMES_OUT","STATEDUMP_AT","STATEDUMP_OUT")) {
    [Environment]::SetEnvironmentVariable($k,$null)
  }
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$PMX)
}

if ($stage -eq "hairfid") {
  Set-Location "$ROOT\tools\hairfid"
  [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
  foreach ($fm in @("0","1")) {
    foreach ($lv in @("1","2","0")) {
      Clear-Env
      [Environment]::SetEnvironmentVariable("MMD_TEST_HAIRCSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
      [Environment]::SetEnvironmentVariable("LEVER",$lv)
      if ($fm -eq "1") { [Environment]::SetEnvironmentVariable("FRICMUL","1") }
      Write-Output ("===== hairfid  LEVER=" + $lv + "  FRICMUL=" + $fm)
      $o = & ".\bin\Release\net9.0\HairFid.exe" 2>&1
      $o | Select-String "^\[全体\] 位置差|^\[静区間\]|^\[ターン窓\]|^\[自前 深貫入|^\[MMD 深貫入"
    }
  }
}

if ($stage -eq "ts") {
  Set-Location "$ROOT\tools\diagnostics\tsbaseline"
  foreach ($lv in @("1","2","0")) {
    Clear-Env
    [Environment]::SetEnvironmentVariable("MMD_TEST_BONECSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose.csv")
    [Environment]::SetEnvironmentVariable("LEVER",$lv)
    Write-Output ("===== tsbaseline/bonecheck  LEVER=" + $lv)
    $o = & ".\bin\Release\net9.0\TsBase.exe" 2>&1
    $o | Select-String "12窓比|平時傾き|ビット" | Select-Object -First 6
  }
}

if ($stage -eq "minnet") {
  Set-Location "$ROOT\tools\diagnostics\restosc"
  foreach ($lv in @("1","2","0")) {
    Clear-Env
    [Environment]::SetEnvironmentVariable("MINNET","1")
    [Environment]::SetEnvironmentVariable("LEVER",$lv)
    Write-Output ("===== minnet  LEVER=" + $lv)
    $o = & ".\bin\Release\net9.0\RestOsc.exe" 2>&1
    $o | Select-Object -Last 10
  }
}

if ($stage -eq "static") {
  Set-Location "$ROOT\tools\diagnostics\restosc"
  $targets = @(
   @{m="Tda-スカート";  p="C:/Users/masa_/Downloads/Tda式初音ミクV4X_Ver1.00/Tda式初音ミクV4X_Ver1.00/Tda式初音ミクV4X_Ver1.00.pmx"; b="スカート"; ref="0.0059177"},
   @{m="ponpu-スカート";p="C:/Users/masa_/Downloads/ぽんぷ長式初音ミク_テスト版_4/ぽんぷ長式初音ミク_テスト版_4/ぽんぷ長式初音ミク.pmx"; b="スカート"; ref="0.0059177"},
   @{m="IA-髪";        p="C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx";        b="髪"; ref="0.000537463"},
   @{m="ettc-髪";      p="C:\mytask2\unity-bullet-physics\Assets\testdata\IA_ettc髪.pmx"; b="髪"; ref="0.000425027"})
  foreach ($lv in @("1","2","0")) {
    Write-Output ("===== 静止(bonedp)  LEVER=" + $lv)
    foreach ($t in $targets) {
      Clear-Env
      [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$t.p)
      [Environment]::SetEnvironmentVariable("BONEDP","1")
      [Environment]::SetEnvironmentVariable("BODIES",$t.b)
      [Environment]::SetEnvironmentVariable("REFFLOOR",$t.ref)
      [Environment]::SetEnvironmentVariable("LEVER",$lv)
      $o = & ".\bin\Release\net9.0\RestOsc.exe" 2>&1
      $rows = $o | Select-String "出荷既定|旧エンジン"
      Write-Output ("  -- " + $t.m)
      $rows | ForEach-Object { Write-Output ("     " + $_.ToString().Trim()) }
    }
  }
}

if ($stage -eq "drivedp") {
  Set-Location "$ROOT\tools\bonecheck"
  foreach ($lv in @("1","2","0")) {
    Clear-Env
    [Environment]::SetEnvironmentVariable("DRIVEDP","1"); [Environment]::SetEnvironmentVariable("AB","none")
    [Environment]::SetEnvironmentVariable("MMD_TEST_BONECSV","C:\mytask2\_mmd_ref\modelA_bone_world_pose_hair.csv")
    [Environment]::SetEnvironmentVariable("LEVER",$lv)
    [Environment]::SetEnvironmentVariable("OUT",("t77_drivedp_lv" + $lv + ".txt"))
    & "C:\Program Files\dotnet\dotnet.exe" run -c Release --project . 2>&1 | Out-Null
    Write-Output ("done drivedp LEVER=" + $lv)
  }
}

if ($stage -eq "sweep") {
  Set-Location "$ROOT\tools\diagnostics\restosc"
  foreach ($lv in @("1","2","0")) {
    Clear-Env
    [Environment]::SetEnvironmentVariable("MODELS","models.txt")
    [Environment]::SetEnvironmentVariable("LEVER",$lv)
    [Environment]::SetEnvironmentVariable("OUT",("t77_sweep_lv" + $lv + ".txt"))
    & ".\bin\Release\net9.0\RestOsc.exe" 2>&1 | Select-String "NaN|発散|完了" | Select-Object -First 3
    Write-Output ("done sweep LEVER=" + $lv)
  }
}
