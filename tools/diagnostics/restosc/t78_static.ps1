Set-Location C:\mytask2\unity-bullet-physics\tools\diagnostics\restosc
# タスク78: 静止3つ組。出荷既定 / +腕長ゲート5 / FRICMUL=1+ゲート5 の3構成。
#   BoneDp の A/B は 旧エンジン vs 出荷既定 なので、読むのは『出荷既定』行。
$targets = @(
 @{m="Tda-スカート";  p="C:/Users/masa_/Downloads/Tda式初音ミクV4X_Ver1.00/Tda式初音ミクV4X_Ver1.00/Tda式初音ミクV4X_Ver1.00.pmx"; b="スカート"; ref="0.0059177"},
 @{m="ponpu-スカート";p="C:/Users/masa_/Downloads/ぽんぷ長式初音ミク_テスト版_4/ぽんぷ長式初音ミク_テスト版_4/ぽんぷ長式初音ミク.pmx"; b="スカート"; ref="0.0059177"},
 @{m="IA-髪";        p="C:\mytask2\unity-bullet-physics\Assets\testdata\IA.pmx";        b="髪"; ref="0.000537463"},
 @{m="ettc-髪";      p="C:\mytask2\unity-bullet-physics\Assets\testdata\IA_ettc髪.pmx"; b="髪"; ref="0.000425027"})
$cfgs = @(
 @{n="出荷既定";          fm=$null; g=$null},
 @{n="出荷既定+ゲート5";   fm=$null; g="5"},
 @{n="FRICMUL=1+ゲート5";  fm="1";   g="5"})
foreach ($c in $cfgs) {
  Write-Output ("===== " + $c.n)
  foreach ($t in $targets) {
    foreach ($k in @("CMARGIN","MODELS","MINNET","DRIVEDP","FRAMES","SPRINGCLAMP","SPRINGMOTOR","NETDUMP","KEEPBODY","EXTRABODIES","INITSTATE","JOINTORDER","ROWTRACE","CONTACTCSV","OUTDIR","SMOKE","SLOP","MANSTATS","ITERS","JOINTS_FIRST","NOCONTACT","ANCHORDUMP","REPLAY","BAUM","CSET","CPOOL","FRICALIGN","FRICMUL","LEVERGATE","ANGCONV","AXES","LEVER","CTHRESH","ROTEXP","NORMFIRST","CRHS","CMAN","LIMGATE","SYMDIST")) { [Environment]::SetEnvironmentVariable($k,$null) }
    [Environment]::SetEnvironmentVariable("FRICMUL",$c.fm)
    [Environment]::SetEnvironmentVariable("LEVERGATE",$c.g)
    [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$t.p)
    [Environment]::SetEnvironmentVariable("BONEDP","1"); [Environment]::SetEnvironmentVariable("BODIES",$t.b)
    [Environment]::SetEnvironmentVariable("REFFLOOR",$t.ref)
    $o = & ".\bin\Release\net9.0\RestOsc.exe" 2>&1
    $i = ($o | Select-String "静止判定の3つ組" | Select-Object -First 1).LineNumber
    $p2 = $o[$i+2].ToString().Trim() -split "\s+"
    Write-Output ("  {0,-14} 収束={1,-6} tau={2,-5} フロア={3,-12} 参照比={4}" -f $t.m,$p2[1],$p2[2],$p2[3],$p2[4])
  }
}
