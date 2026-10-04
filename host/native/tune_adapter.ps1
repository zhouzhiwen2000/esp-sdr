#Requires -RunAsAdministrator
[CmdletBinding()]
param([string]$Name='Ethernet 8', [switch]$Restore)
$ErrorActionPreference='Stop'
$adapter=Get-NetAdapter -Name $Name
if($adapter.InterfaceDescription -notlike '*Realtek USB GbE*') {
    throw "Expected the S31 direct-link Realtek USB GbE adapter; found $($adapter.InterfaceDescription)"
}
# Only the receiving side of this dedicated adapter is changed. Driver's declared ranges:
# ReceiveBufferLen: 1..256 (original 16); PendingReceives: 1..64 (original 6).
$buffers=if($Restore){16}else{256}
$urbs=if($Restore){6}else{64}
try {
    $null=Invoke-RestMethod -Uri 'http://localhost:8765/api/stop' -Method Post -ContentType 'application/json' -Body '{}' -TimeoutSec 5
} catch { }
Get-NetAdapterAdvancedProperty -Name $Name -RegistryKeyword ReceiveBufferLen,PendingReceives |
    Select-Object DisplayName,RegistryValue | Format-Table
Set-NetAdapterAdvancedProperty -Name $Name -RegistryKeyword ReceiveBufferLen -RegistryValue $buffers -NoRestart
Set-NetAdapterAdvancedProperty -Name $Name -RegistryKeyword PendingReceives -RegistryValue $urbs -NoRestart
Restart-NetAdapter -Name $Name -Confirm:$false
Start-Sleep -Seconds 3
Get-NetAdapterAdvancedProperty -Name $Name -RegistryKeyword ReceiveBufferLen,PendingReceives |
    Select-Object DisplayName,RegistryValue | Format-Table
Get-NetAdapter -Name $Name | Select-Object Name,Status,LinkSpeed
