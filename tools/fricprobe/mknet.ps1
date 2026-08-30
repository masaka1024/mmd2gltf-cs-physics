# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $REPO\tools\fricprobe
foreach ($p in (Get-ChildItem *.pmx | Sort-Object Name)) {
  foreach ($k in @("CMARGIN","MODELS","MINNET","DRIVEDP","SMOKE","BONEDP","BODIES","REFFLOOR","SLOP","JOINTORDER","NOCONTACT","INITSTATE","ANCHORDUMP","ROWTRACE","BAUM","REPLAY","MANSTATS","OUT","CSET","NORMFIRST","CPOOL","FRICALIGN","FRICMUL","CONTACTCSV")) { [Environment]::SetEnvironmentVariable($k,$null) }
  [Environment]::SetEnvironmentVariable("MMD_TEST_PMX",$p.FullName)
  [Environment]::SetEnvironmentVariable("NETDUMP","1"); [Environment]::SetEnvironmentVariable("KEEPBODY","box")
  [Environment]::SetEnvironmentVariable("EXTRABODIES","slope,box")
  [Environment]::SetEnvironmentVariable("SUBSTEPS","2"); [Environment]::SetEnvironmentVariable("ITERS","10"); [Environment]::SetEnvironmentVariable("FRAMES","2")
  [Environment]::SetEnvironmentVariable("OUTDIR",("net_" + $p.BaseName))
  & "$REPO\tools\diagnostics\restosc\bin\Release\net9.0\RestOsc.exe" 2>&1 | Out-Null
}
Write-Output done
