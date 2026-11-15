# Ubuntu-24.04 WSL2 runtime (Mesa d3d12 EGL -> RTX 5050). Not Docker Desktop.
#   powershell -File scripts/run_gpu.ps1
#   powershell -File scripts/run_gpu.ps1 pytest -q
#   powershell -File scripts/run_gpu.ps1 ruff check .
#   powershell -File scripts/run_gpu.ps1 --% physics cell-run --scenario human_stop --seed 0 --headless --no-record --out results/demo/check-hstop-wsl
#   powershell -File scripts/run_gpu.ps1 --% cell-run --scenario human_stop --seed 0 --headless --video results/demo/full-mission.mp4
# Use --% so PowerShell does not eat --out / --video as its own parameters.
$ErrorActionPreference = "Stop"
$winRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$drive = $winRoot.Substring(0, 1).ToLower()
$rest = $winRoot.Substring(2).Replace("\", "/")
$wslRoot = "/mnt/$drive$rest"
$script = "$wslRoot/scripts/run_gpu.sh"
& wsl -d Ubuntu-24.04 -u root -- sed -i "s/\r$//" $script
if ($args -and $args.Count -gt 0) {
    & wsl -d Ubuntu-24.04 -u root -- bash $script @args
} else {
    & wsl -d Ubuntu-24.04 -u root -- bash $script
}
exit $LASTEXITCODE
