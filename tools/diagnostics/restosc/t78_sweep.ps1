# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
Set-Location $REPO\tools\diagnostics\restosc
# タスク78 採用後: 35モデルスイープで NaN/発散が出ないことを確認する。
foreach ($c in @(@{n="旧出荷既定";fm="0";g="0"}, @{n="新出荷既定";fm=$null;g=$null})) {
  foreach ($k in @("CMARGIN","MINNET","DRIVEDP","BONEDP","BODIES","REFFLOOR","FRAMES","NETDUMP","KEEPBODY","EXTRABODIES","INITSTATE","JOINTORDER","ROWTRACE","CONTACTCSV","OUTDIR","SMOKE","SLOP","MANSTATS","ITERS","SUBSTEPS","JOINTS_FIRST","NOCONTACT","ANCHORDUMP","REPLAY","BAUM","CSET","CPOOL","FRICALIGN","NORMFIRST","ANGCONV","AXES","LEVER","CTHRESH","ROTEXP","CRHS","CMAN","LIMGATE","SYMDIST")) { [Environment]::SetEnvironmentVariable($k,$null) }
  [Environment]::SetEnvironmentVariable("FRICMUL",$c.fm)
  [Environment]::SetEnvironmentVariable("LEVERGATE",$c.g)
  [Environment]::SetEnvironmentVariable("MODELS","models.txt")
  [Environment]::SetEnvironmentVariable("OUT",("t78_sweep_" + $c.n + ".txt"))
  & ".\bin\Release\net9.0\RestOsc.exe" 2>&1 | Out-Null
  Write-Output ("done " + $c.n)
}
