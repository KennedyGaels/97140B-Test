# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Kennedy Gaels 97140B                                         #
# 	Created:      Sat Sep 28 2023                                              #
# 	Description:  Bot with sticking battery                                    #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

################################################################################    declare hardware devices

con=Controller()
fl=Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
blb=Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
blt=Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
fr=Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
brb=Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
brt=Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
cata=Motor(Ports.PORT20, GearSetting.RATIO_36_1, True)
intake=Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)
rs=Rotation(Ports.PORT9)
imu=Inertial(Ports.PORT10)
brain=Brain()
######################################################################################  driver control

bite=8

def drive_hold():
    fl.stop(HOLD)
    blb.stop(HOLD)
    blt.stop(HOLD)
    fr.stop(HOLD)
    brb.stop(HOLD)
    brt.stop(HOLD)

def fire():                                                                         #making the cata a def to start it on button release
    cata.spin(FORWARD, 12, VOLT)
    wait(400)
    while rs.angle() > 6:                                                           #this number should be between 6 and 6.5 for now
        cata.spin(FORWARD, 10, VOLT)
        wait(10)
    cata.stop(COAST)

def continuous_fire():
    cata.spin(FORWARD, 10, VOLT)

def stop_cata():
    cata.stop(COAST)

def open():
    cata.spin(FORWARD, 10, VOLT)
    wait(200)
    cata.stop(COAST)

def forwards(bite):
    fl.spin(FORWARD, bite, VOLT)
    blb.spin(FORWARD, bite, VOLT)
    blt.spin(FORWARD, bite, VOLT)
    fr.spin(FORWARD, bite, VOLT)
    brb.spin(FORWARD, bite, VOLT)
    brt.spin(FORWARD, bite, VOLT)

def backwards(bite):
    fl.spin(REVERSE, bite, VOLT)
    blb.spin(REVERSE, bite, VOLT)
    blt.spin(REVERSE, bite, VOLT)
    fr.spin(REVERSE, bite, VOLT)
    brb.spin(REVERSE, bite, VOLT)
    brt.spin(REVERSE, bite, VOLT)

def leftbank(bite):
    fl.spin(FORWARD, bite, VOLT)
    blb.spin(FORWARD, bite, VOLT)
    blt.spin(FORWARD, bite, VOLT)
    fr.spin(FORWARD, bite/3, VOLT)
    brb.spin(FORWARD, bite/3, VOLT)
    brt.spin(FORWARD, bite/3, VOLT)

def rightbank(bite):
    fl.spin(FORWARD, bite/3, VOLT)
    blb.spin(FORWARD, bite/3, VOLT)
    blt.spin(FORWARD, bite/3, VOLT)
    fr.spin(FORWARD, bite, VOLT)
    brb.spin(FORWARD, bite, VOLT)
    brt.spin(FORWARD, bite, VOLT)

def cstop():
    fl.stop(COAST)
    blb.stop(COAST)
    blt.stop(COAST)
    fr.stop(COAST)
    brb.stop(COAST)
    brt.stop(COAST)

def hstop():
    fl.stop(HOLD)
    blb.stop(HOLD)
    blt.stop(HOLD)
    fr.stop(HOLD)
    brb.stop(HOLD)
    brt.stop(HOLD)

################################################################################    uses IMU to calculate spin point
# read https://georgegillard.com/resources/documents
# "An introduction to PID controllers"
# and  https://renegaderobotics.org/pid-beginners-guide/  before venturing here
################################################################################ 

def gyrospin(setPoint):                                                             #spins the robot using values from the inertial sensor

    nowPoint = imu.rotation()
    error = setPoint - nowPoint                                                     #gets the starting error just in case
    #ratio = error/20                                                               #not using it now, tried it to scale power
    errorTotal = 0                                                                  #initialize variable
    errorLast = 0                                                                   #initialize variable
    kP = 0.35                                                                      #tunable proportioanal parameter
    kD = 0                                                                          #tunable derivative parameter
    kI = 0                                                                          #tunable integral parameter
    treshold = 0.2                                                                  #error treshold (precision)
    minturnspeed = 5                                                                #a minimum speed to avoid stall

    brain.screen.clear_screen(Color.BLUE)                                           #lets the entire world know we are spinning

    while(abs(error) > treshold):                                                   #stop at threshold so we don't calculate infinitely to the perfect angle

        brain.screen.print_at(imu.rotation(), x=5, y=50)
        
        nowPoint = imu.rotation()
        error = setPoint - nowPoint                                                 #calculate error
        errorTotal += error                                                         #accumulated error (for integral)
        errorLast = error                                                           #previous error (for derivative) 

        pTerm = error * kP                                                          #proportional term
        iTerm = kI * errorTotal                                                     #integral term
        dTerm = kD * (error - errorLast)                                            #derivative term
        
        if error>0:
            power = max(minturnspeed,(pTerm + iTerm + dTerm))                       #the new power for motors when under
        else:                                                                       #using a minimum speed instead of 0 to not stall
            power = min(-minturnspeed,(pTerm + iTerm + dTerm))                      #the new power for motors when over

        fl.spin(REVERSE, power, PERCENT)
        fr.spin(FORWARD, power, PERCENT)                                            #run all 4 motors with above power                                      
        blt.spin(REVERSE, power, PERCENT)
        brt.spin(FORWARD, power, PERCENT)
        blb.spin(REVERSE, power, PERCENT)
        brb.spin(FORWARD, power, PERCENT)

        wait(10)                                                                    #so the brain does not get a headache 

        #print("imu %5.1f" %nowPoint,"er %5.1f" %error,"   P %5.1f" %pTerm,"   pw %5.2f" %power)
                                                                                    #a simple debug print to console

    brain.screen.clear_screen(Color.BLACK)                                          #returns background to black when done
    drive_hold()        

def imu_calibrate():
    brain.screen.draw_image_from_file("brainscreen.png", 0,0)
    imu.calibrate()
    brain.screen.set_pen_color(Color.YELLOW)
    brain.screen.print_at("CALIBRATING IMU",x=250, y=50)
    brain.screen.print_at("NO TOUCH OR YOU DIE",x=250, y=75)
    wait(2000)

def catatime(cata_time):
    cata.spin(FORWARD, 11, VOLT)
    wait(cata_time)
    cata.stop(COAST)

#dont use this def
def autoright():
    imu.calibrate
    wait(2000)
    while imu.rotation() < 15:
        fl.spin(REVERSE, 8, VOLT)
        blb.spin(REVERSE, 8, VOLT)
        blt.spin(REVERSE, 8, VOLT)
        fr.spin(FORWARD, 8, VOLT)
        brb.spin(FORWARD, 8, VOLT)
        brt.spin(FORWARD, 8, VOLT)
    fl.stop(HOLD)
    blb.stop(HOLD)
    blt.stop(HOLD)
    fr.stop(HOLD)
    brb.stop(HOLD)
    brt.stop(HOLD)
    
    wait(1000)
    open()
    
    fl.spin(REVERSE, 8, VOLT)
    blb.spin(REVERSE, 8, VOLT)
    blt.spin(REVERSE, 8, VOLT)
    fr.spin(FORWARD, 8, VOLT)
    brb.spin(FORWARD, 8, VOLT)
    brt.spin(FORWARD, 8, VOLT)
    wait(4000)
    fl.stop(HOLD)
    blb.stop(HOLD)
    blt.stop(HOLD)
    fr.stop(HOLD)
    brb.stop(HOLD)
    brt.stop(HOLD)

    wait(2000)

    fl.spin(FORWARD, 8, VOLT)                                                       #move forward into bar
    blb.spin(FORWARD, 8, VOLT)
    blt.spin(FORWARD, 8, VOLT)
    fr.spin(FORWARD, 8, VOLT)
    brb.spin(FORWARD, 8, VOLT)
    brt.spin(FORWARD, 8, VOLT)
    wait(4000)
    fl.stop(COAST)
    blb.stop(COAST)
    blt.stop(COAST)
    fr.stop(COAST)
    brb.stop(COAST)
    brt.stop(COAST)
#end of def not to use

imu_calibrate()
touch_x = 0
touch_y = 0
gray = Color(140,140,140)
gray2 = Color(100,100,100)
press = 0
run = True
auton_select = "-"

while run==True:                                                                    #print menu                                              #Main menu
    brain.screen.set_pen_width(1)
    brain.screen.set_fill_color(Color.TRANSPARENT)
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.draw_image_from_file("brainscreen3.png", 0,0)
    brain.screen.draw_rectangle(220,60,110,50) 
    brain.screen.draw_rectangle(220,170, 110, 50)
    brain.screen.draw_rectangle(370,60, 80, 50)
    brain.screen.draw_rectangle(370,170, 80,50)
    brain.screen.print_at("Auto Far",x=225,y=90)
    brain.screen.print_at("Skills",x=375,y=90)
    brain.screen.print_at("Auto Near",x=225, y=200)
    brain.screen.print_at("Drive", x=375, y=200)


    if brain.screen.pressing():
        touch_x = brain.screen.x_position()
        touch_y = brain.screen.y_position()
        press = 1
    elif press == 1:
        if touch_x <= 350:
            if touch_y <= 120:
                auton_select = "Auto Far"
                run = False
            elif touch_y > 120:
                auton_select = "Auto Near"
                run = False
        elif touch_x > 350:
            if touch_y <= 120:
                auton_select = "Skills"
                run = False
            elif touch_y > 120:
                auton_select = "Drive"
                run = False
        press = 0
    wait(100)

brain.screen.draw_image_from_file("brainscreen.png", 0,0)
brain.screen.set_pen_color(Color.CYAN)
brain.screen.print_at(auton_select, x=350, y=220)

def driver():


    con.buttonR1.released(fire)
    con.buttonA.released(open)
    con.buttonR2.released(continuous_fire)
    con.buttonB.released(stop_cata)

    while True:                                                                         #Drive loop
        axis_3=con.axis3.position()**3/80000                                            #0-12.5V
        axis_1=con.axis1.position()**3/80000                                            #0-6ish to make turns less crazy
        fl.spin(FORWARD, axis_3-axis_1, VOLT)                                           #direct voltage drive disables velocity PID
        blb.spin(FORWARD, axis_3-axis_1, VOLT)                                           #shuould reduce overheating
        blt.spin(FORWARD, axis_3-axis_1, VOLT)
        fr.spin(FORWARD, axis_3+axis_1, VOLT)
        brb.spin(FORWARD, axis_3+axis_1, VOLT)
        brt.spin(FORWARD, axis_3+axis_1, VOLT)

        #if con.buttonR2.pressing():
        #    cata.spin(FORWARD, 12, VOLT)
        #else: 
        #    cata.stop(COAST)

        if con.buttonL1.pressing():
            intake.spin(FORWARD, 12, VOLT)
        elif con.buttonL2.pressing():
            intake.spin(REVERSE, 12, VOLT)
        else:
            intake.stop(COAST)

        tmpcata=cata.temperature(TemperatureUnits.CELSIUS)
        brain.screen.print_at(tmpcata, x=240,y=120)
        if tmpcata>=45:
            brain.screen.clear_screen(Color.RED)
            brain.screen.print_at(tmpcata, x=240,y=120)

#start of select defs
def auton():
    if auton_select == 'Auto Far':
        imu.calibrate
        wait(2000)
        backwards(8)
        wait(700)
        cstop()
        wait(300)
        gyrospin(85)
        intake.spin(REVERSE, 12, VOLT)
        wait(1000)
        intake.stop(COAST)
        backwards(12)
        wait(400)
        cstop()

    if auton_select == 'Auto Near':
        imu.calibrate
        wait(2000)
        gyrospin(30)
        wait(1000)
        open()

        fl.spin(REVERSE, 8, VOLT)
        blb.spin(REVERSE, 8, VOLT)
        blt.spin(REVERSE, 8, VOLT)
        fr.spin(FORWARD, 8, VOLT)
        brb.spin(FORWARD, 8, VOLT)
        brt.spin(FORWARD, 8, VOLT)
        wait(230)
        hstop()

        wait(1000)
        forwards(8)
        wait(400)
        cstop()

    if auton_select == 'Skills':
        imu.calibrate
        wait(2000)
        forwards(5)
        wait(500)
        cstop()
        wait(300)
        gyrospin(80)
        wait(200)
        backwards(5)
        wait(500)
        cstop()
        wait(300)
        catatime(3) #30000
        wait(300)
        forwards(5)
        wait(500)
        cstop()
        wait(300)
        gyrospin(180)
        wait(200)
        forwards(5)
        wait(500)
        cstop()
        wait(300)
        gyrospin(100)
        wait(300)
        forwards(5)
        wait(2300)
        leftbank(5)
        wait(630)
        cstop()
        wait(300)
        forwards(12)
        wait(700)
        cstop()
        wait(300)
        backwards(5)
        wait(500)
        cstop()
        wait(200)
        forwards(12)
        wait(500)
        cstop()
        wait(300)
        backwards(5)
        wait(300)
        cstop()
        wait(300)
        gyrospin(270)
        wait(300)
        rightbank(5)
        wait(1000)
        cstop()
        #wait(300)
        #leftbank(5)
        #wait(300)
        #gyrospin(90)
        #forwards(12)
        #wait(500)
        #cstop()
        #wait(300)
        #backwards(5)
        #wait(300)
        #cstop()
        #wait(300)
        #gyrospin(90)
        #wait(300)
        #forwards(5)
        #wait(300)
        #gyrospin(90)
        #wait(300)
        #forwards(12)
        #wait(500)
        #cstop()
        #wait(300)
        #backwards(5)
        #wait(300)
        #cstop()
        #wait(300)




comp = Competition(driver, auton)                                                       #must be here for driver and auton for vex comp switch
