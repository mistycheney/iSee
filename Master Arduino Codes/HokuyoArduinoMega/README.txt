1. Put USB shield on Arduino. Possibly need external power, either use a power jack or power supply with 6-6.5v to Vin and 0 to GND.
2. Connect hokuyo to USB shield.
3. Upload example code /USBHostShield2/example/acm/acm_terminal/acm_terminal.pde
4. Open Serial Monitor. You should see "ACM configured". If not, need more power.
5. In Serial Monitor, send "VV" followed by a newline character.
6. You should see the response, which contains the version information of the lidar.
7. Send 'GD0044072501', and you should see a response containing the encoded range data.


Note: 
1. More command can be found here:
http://www.hokuyo-aut.jp/02sensor/07scanner/download/urg_programs_en/scip_commands_page.html

