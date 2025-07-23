$pro_mess_ram_hex = "0x20000"
$pro_mess_rom_hex = "0x100000"

$map_file =Get-ChildItem -Filter "*.map"| Select-Object -ExpandProperty Name #获取当前目录下的.MAP文件
#Write-Host $map_file

$map_mess_ram =Get-Content -Path $map_file|Select-String -Pattern "Total RW Size" #获取.MAP文件中RAM信息行
Write-Host $map_mess_ram
$map_mess_rom =Get-Content -Path $map_file|Select-String -Pattern "Total ROM Size" #获取.MAP文件中ROM信息行
Write-Host $map_mess_rom
Write-Host " "
$pattern = "\d+" #获取.MAP文件中RAM信息行中的数字
$matches = [regex]::Match($map_mess_ram, $pattern)
$map_ram_num = $matches.Value
#Write-Host $map_ram_num
$matches = [regex]::Match($map_mess_rom, $pattern) #获取.MAP文件中ROM信息行中的数字
$map_rom_num = $matches.Value
#Write-Host $map_rom_num


$pro_ram_num = [convert]::ToInt32($pro_mess_ram_hex, 16)
#Write-Host $pro_ram_num
$pro_rom_num = [convert]::ToInt32($pro_mess_rom_hex, 16)
#Write-Host $pro_rom_num

$ram_per =($map_ram_num /$pro_ram_num)*100
$ram_per = [math]::Round($ram_per, 2)
$ram_per_txt=$ram_per.ToString()+"% RAM USED"

$ram_per_int=[int]$ram_per
$ram_per_show =" "
for ($i = 1; $i -le $ram_per_int; $i++) {
$ram_per_show=$ram_per_show+"█"
}
for ($i = 1; $i -le (100-$ram_per_int); $i++) {
$ram_per_show=$ram_per_show+"≡"
}
$ram_per_show=" "+$ram_per_show+" "+$ram_per_txt
Write-Host $ram_per_show
Write-Host " "

$rom_per =($map_rom_num /$pro_rom_num)*100
$rom_per = [math]::Round($rom_per, 2)
$rom_per_txt=$rom_per.ToString()+"% ROM USED"

$rom_per_int=[int]$rom_per
$rom_per_show =" "
for ($i = 1; $i -le $rom_per_int; $i++) {
$rom_per_show=$rom_per_show+"█"
}
for ($i = 1; $i -le (100-$rom_per_int); $i++) {
$rom_per_show=$rom_per_show+"≡"
}
$rom_per_show=" "+$rom_per_show+" "+$rom_per_txt
Write-Host $rom_per_show