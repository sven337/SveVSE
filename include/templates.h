const char SRC_CONFIG_TEMPLATE[] PROGMEM = 
"{\"configversion\":1,\"wifi\":{\"bssid\":\"\",\"ssid\":\"EVSE-WiFi\",\"wmode\":true,\"pswd\":\"\",\"staticip\":false,\"ip\":\"\",\"subnet\":\"\",\"gateway\":\"\",\"dns\":\"\"},\"meter\":[{\"usemeter\":false,\"metertype\":\"S0\",\"price\":0,\"kwhimp\":1000,\"implen\":30,\"meterphase\":1,\"factor\":1}],\"rfid\":{\"userfid\":false,\"rfidgain\":32},\"ntp\":{\"ntpip\":\"pool.ntp.org\",\"timezone\":1},\"button\":[{\"usebutton\":true}],\"system\":{\"hostnm\":\"evse-wifi\",\"adminpwd\":\"adminadmin\",\"wsauth\":true,\"debug\":false,\"maxinstall\":16,\"evsecount\":1,\"logging\":true,\"api\":true},\"evse\":[{\"mbid\":1,\"alwaysactive\":false,\"disableled\":false,\"resetcurrentaftercharge\":true,\"evseinstall\":16,\"avgconsumption\":15,\"rseactive\":false,\"rsevalue\":80}]}";
