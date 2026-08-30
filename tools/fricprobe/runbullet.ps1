# リポジトリ直下を自分の位置から導出する (絶対パスを書かない)
$REPO = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $REPO\tools\diagnostics\bulletref
Write-Output "  == 純Bullet (bulletref) =="
foreach ($d in (Get-ChildItem "$REPO\tools\fricprobe\net_fric_*" -Directory | Sort-Object Name)) {
  $out = "$REPO\tools\fricprobe\bul_" + $d.Name
  New-Item -ItemType Directory -Force $out | Out-Null
  & ".\bulletref.exe" --net ($d.FullName + "\net.txt") --out $out --frames 120 --substeps 2 --iters 10 --erp 0.2 --keepmargin 2>&1 | Out-Null
}
Write-Output done
