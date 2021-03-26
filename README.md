# GrowingLight

Project created to maintain ESP8266 board with different sensors.

This project is specifecly build to interact with Blynk server

All settings can be set in Blynk application, except application key, network ssid and password.
These settings now shoud be entered in code. In the future versions there may be a fix for it, but chances are really low.

# Application interface
## Main page
<img src="/readmeSrc/settingsPage.jpg" height="500px">

Here you can find such air and soil parametrs as temperature and himidity. You can control light and pump work from here.

## Settings page
<img src="/readmeSrc/mainPage.jpg" height="500px">

Here you can find any settings for the application. 

## Modes
Light mode control - use automatic light switching or only manual. 

Watering mode control - use semi automatic mode or only manual.

### Logic explanations
#### Watering logic

Here, I guess, you need an explanation of how semi-automatic mode works.
Soil humidity can be in 3 zones - red (up to 50% humidity), yellow (from 50% up to 75%) and green (75% or more).
On the Settings page you can specify for how long you want to turn on the pump using the "watering time multiplier" slider.
The duration depends on the difference in humidity between current value and 75% and is multiplied by calculated value, including "watering time multiplier".
If the current humidity level is in "green" zone and you using semi-automatic mode, the pump won't turn on.
In other zones, it will turn on, work for the specified time and turn off.

#### Light logic

Everything is very simple here. You can specify border value using the "light level border (lux)" slider on the Settings page.
If light level is higher than border value, the lamp doesn't turn on. If the light level is lower than border value, the lamp turns on.
