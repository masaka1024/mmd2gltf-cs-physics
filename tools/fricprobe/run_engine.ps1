# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $REPO\tools\fricprobe
$conds = @(@{n="既定(幾何平均)"; fm="0"}, @{n="FRICMUL(積)"; fm="1"})
foreach ($c in $conds) {
 foreach ($p in (Get-ChildItem *.pmx | Sort-Object Name)) {
  foreach ($k in @("CMARGIN","MODELS","MINNET","DRIVEDP","SMOKE","BONEDP","BODIES","REFFLOOR","SLOP","JOINTORDER","NOCONTACT","INITSTATE","ANCHORDUMP","ROWTRACE","BAUM","REPLAY","MANSTATS","OUT","CSET","NORMFIRST","CPOOL","FRICALIGN","CONTACTCSV")) { [Environment]::SetEnvironmentVariable($k,$null) }
  [Environment]::SetEnvironmentVariable("FRICMUL",$c.fm)
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$p.FullName)
  [Environment]::SetEnvironmentVariable("NETDUMP","1"); [Environment]::SetEnvironmentVariable("KEEPBODY","box")
  [Environment]::SetEnvironmentVariable("EXTRABODIES","slope")
  [Environment]::SetEnvironmentVariable("SUBSTEPS","2"); [Environment]::SetEnvironmentVariable("ITERS","10"); [Environment]::SetEnvironmentVariable("FRAMES","120")
  [Environment]::SetEnvironmentVariable("OUTDIR",("out_fm" + $c.fm + "_" + $p.BaseName))
  & "$REPO\tools\diagnostics\restosc\bin\Release\net9.0\RestOsc.exe" 2>&1 | Out-Null
 }
 Write-Output ("done " + $c.n)
}
