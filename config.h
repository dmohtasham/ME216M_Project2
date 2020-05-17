/******************************* WIFI **************************************/
// Fill in your wifi SSID and password below
const char* wifi_ssid = "Fios-QZYHK";  // network name -- put your WiFi SSID here
const char* wifi_password = "away9062bay996mash";     // network password -- enter your WiFi password (if one is needed)

/******************** Advice Slip API Config ***************************/
// Uses Advice Slip API to generate random advice.
// In accordance with the Advice Slip API. See https://api.adviceslip.com/ for details.
const String ad_host = "api.openweathermap.org";  // host for advice API (excluding https://)
const String ad_path = "/data/2.5/onecall?lat=38.9012&lon=77.2653&units=imperial&exclude=minutely,hourly&appid=a8ba938aa78f3c26e7aff548c17dbeba"; // path for generating a random piece of advice (starting with a slash)
const int ad_port = 443;
