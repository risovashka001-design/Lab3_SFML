#requires -Version 5.1
# Automatically install official SFML 2.6.2 VC17 x64 with SHA-256 verification.
# No CMake, no older project, no administrator rights.
param([string]$LocalZip = '')
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
Add-Type -AssemblyName System.IO.Compression.FileSystem
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$dest = Join-Path $root 'third_party\SFML'
$package = 'SFML-2.6.2-windows-vc17-64-bit.zip'
$expectedHash = 'f5995724604d74d06efe89544315d11ac2d3c2d348854c6be55a7a11fb7ecad8'
$required = @('include\SFML\Graphics.hpp', 'lib\sfml-graphics.lib', 'lib\sfml-graphics-d.lib',
              'lib\sfml-system.lib', 'lib\sfml-system-d.lib', 'lib\sfml-window.lib', 'lib\sfml-window-d.lib',
              'bin\sfml-graphics-2.dll','bin\sfml-graphics-d-2.dll',
              'bin\sfml-window-2.dll','bin\sfml-window-d-2.dll',
              'bin\sfml-system-2.dll','bin\sfml-system-d-2.dll')
function ValidSDK([string]$dir) {
    if (-not (Test-Path -LiteralPath $dir -PathType Container)) { return $false }
    foreach($item in $required) { if(-not (Test-Path -LiteralPath (Join-Path $dir $item) -PathType Leaf)) {return $false} }
    return $true
}
function ValidZip([string]$file) {
    try {
        if (-not (Test-Path -LiteralPath $file -PathType Leaf)) {return $false}
        if ((Get-Item -LiteralPath $file).Length -lt 15000000) {return $false}
        $hash=(Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($hash -ne $expectedHash) {return $false}
        $zip=[IO.Compression.ZipFile]::OpenRead($file)
        try {
            $entries=@($zip.Entries | ForEach-Object {$_.FullName.Replace('/','\')})
            foreach($fragment in @('include\SFML\Graphics.hpp','lib\sfml-graphics-d.lib','bin\sfml-window-2.dll')) {
                if (-not ($entries | Where-Object {$_ -like "*$fragment"})) {return $false}
            }
            return $true
        } finally {$zip.Dispose()}
    } catch {return $false}
}
function InstallArchive([string]$archive) {
    $tmp=Join-Path $root '.sfml-extract-temp'
    if (Test-Path -LiteralPath $tmp) {Remove-Item -LiteralPath $tmp -Recurse -Force}
    try {
        New-Item -ItemType Directory -Path $tmp -Force | Out-Null
        [IO.Compression.ZipFile]::ExtractToDirectory($archive, $tmp)
        $sdk=$null
        foreach($p in @($tmp, (Join-Path $tmp 'SFML-2.6.2'))) {if(ValidSDK $p){$sdk=$p;break}}
        if(-not $sdk) {throw 'The downloaded ZIP does not contain the expected SFML include/lib/bin files.'}
        New-Item -ItemType Directory -Path $dest -Force | Out-Null
        foreach($sub in @('include','lib','bin')) {
            $out=Join-Path $dest $sub
            if (Test-Path -LiteralPath $out) {Remove-Item -LiteralPath $out -Recurse -Force}
            Copy-Item -LiteralPath (Join-Path $sdk $sub) -Destination $out -Recurse -Force
        }
        if (-not (ValidSDK $dest)) {throw 'Installed SDK validation failed.'}
    } finally {
        if (Test-Path -LiteralPath $tmp) {Remove-Item -LiteralPath $tmp -Recurse -Force}
    }
}
function Fetch([string]$url,[string]$to) {
    if (Test-Path -LiteralPath $to) {Remove-Item -LiteralPath $to -Force}
    try {
        Write-Host "  PowerShell download: $url"
        Invoke-WebRequest -Uri $url -UseBasicParsing -OutFile $to -MaximumRedirection 8 -TimeoutSec 120 -ErrorAction Stop
        if (ValidZip $to) {return $true}
        Write-Host '  Download returned wrong content or damaged file.' -ForegroundColor Yellow
    } catch { Write-Host ('  PowerShell: ' + $_.Exception.Message) -ForegroundColor Yellow }
    if (Test-Path -LiteralPath $to) {Remove-Item -LiteralPath $to -Force}
    $curl=Get-Command curl.exe -ErrorAction SilentlyContinue
    if($curl) {
        try {
            Write-Host '  Trying curl.exe ...'
            & $curl.Source '-fSL' '--retry' '2' '--connect-timeout' '20' '--max-time' '180' '--output' $to $url
            if ($LASTEXITCODE -eq 0 -and (ValidZip $to)) {return $true}
        } catch {Write-Host ('  curl: ' + $_.Exception.Message) -ForegroundColor Yellow}
    }
    if (Test-Path -LiteralPath $to) {Remove-Item -LiteralPath $to -Force}
    return $false
}
try {
    Write-Host 'SFML 2.6.2 / Visual Studio 2022 / x64' -ForegroundColor Cyan
    if(ValidSDK $dest) {Write-Host 'SFML already bundled in project. OK.' -ForegroundColor Green;exit 0}
    $candidates=@()
    if($LocalZip){$candidates+= $LocalZip}
    $candidates+=@((Join-Path $root $package), (Join-Path $env:USERPROFILE "Downloads\$package"))
    foreach($file in $candidates) {
        if(ValidZip $file) {Write-Host "Installing verified archive: $file";InstallArchive $file;Write-Host 'SUCCESS' -ForegroundColor Green;exit 0}
    }
    Write-Host 'No existing SFML is needed. Downloading the official SDK automatically...'
    $urls=@(
      ('https://www.sfml-dev.org/files/'+$package),
      ('https://mirror1.sfml-dev.org/files/'+$package),
      ('https://github.com/SFML/SFML/releases/download/2.6.2/'+$package),
      ('https://downloads.sourceforge.net/project/sfml.mirror/2.6.2/'+$package+'?download')
    )
    $temporary=Join-Path $root '.sfml-download.part'
    $success=$false
    foreach ($url in $urls) {if(Fetch $url $temporary){$success=$true;break}}
    if(-not $success) {
        throw ('Automatic SFML download failed for all sources. Open https://www.sfml-dev.org/download/sfml/2.6.2/ and download Visual C++ 17 (2022) 64-bit. Put '+$package+' next to START.bat and retry. No old project is needed.')
    }
    $saved=Join-Path $root $package
    Move-Item -LiteralPath $temporary -Destination $saved -Force
    InstallArchive $saved
    Write-Host 'SUCCESS: official SFML is fully installed into third_party\SFML.' -ForegroundColor Green
    Write-Host ('Archive preserved: '+$saved)
    exit 0
} catch {
    Write-Host ('ERROR: '+$_.Exception.Message) -ForegroundColor Red
    exit 1
}
