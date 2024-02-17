/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Pantograph        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           lastPosEE                           = new PVector(0, 0);
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 
float             dampening                           = 0;

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;
// scene 0
FBox              l1,l2;
// scene 1
FBox              l3;
// scene 2
FBox              l4, wall1, wall2, wall3;


/* define game start */
int           scene                               = 0;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CW, 168, 4096, 2);
  widgetOne.add_encoder(2, CW, 12, 4096, 1);
  
  
  widgetOne.device_set_parameters();
  
  /* Scene 1 */
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
   
  /* Set viscous layer */
  l1                  = new FBox(27,6);
  l1.setPosition(24.5/2,8);
  l1.setFill(150,150,255,80);
  l1.setDensity(100);
  l1.setSensor(true);
  l1.setNoStroke();
  l1.setStatic(true);
  l1.setName("Not Water");
  world.add(l1);
   
  l2                  = new FBox(27,6);
  l2.setPosition(24.5/2,2);
  l2.setFill(150,150,255,80);
  l2.setDensity(30);
  l2.setSensor(true);
  l2.setNoStroke();
  l2.setStatic(true);
  l2.setName("Water");
  world.add(l2);
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);
  
  /* scene 2 */
  l3                  = new FBox(27,12);
  l3.setPosition(24.5/2,5);
  l3.setFill(255,150,150,80);
  l3.setDensity(100);
  l3.setSensor(true);
  l3.setNoStroke();
  l3.setStatic(true);
  l3.setName("Honey");
  
  /* scene 3 */
  l4                  = new FBox(27,12);
  l4.setPosition(24.5/2,5);
  l4.setFill(150,255,150,80);
  l4.setDensity(1);
  l4.setSensor(true);
  l4.setNoStroke();
  l4.setStatic(true);
  l4.setName("Noise");
  // WALLS DON'T WORK BUT I HAVE NO IDEA WHY :D
  // Let's say it's decoration
  wall1                  = new FBox(20.0, 1);
  wall1.setPosition(edgeTopLeftX+worldWidth/2.0, 2); 
  wall1.setFill(150,255,150);
  wall1.setNoStroke();
  wall1.setStaticBody(true);
  
  wall2                  = new FBox(20.0, 1);
  wall2.setPosition(edgeTopLeftX+worldWidth/2.0, 5); 
  wall2.setFill(150,255,150);
  wall2.setNoStroke();
  wall2.setStaticBody(true);
  
  wall3                  = new FBox(20.0, 1);
  wall3.setPosition(edgeTopLeftX+worldWidth/2.0, 8); 
  wall3.setFill(150,255,150);
  wall3.setNoStroke();
  wall3.setHaptic(true);
  wall3.setStaticBody(true);
  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
  
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/
/* keypress section ****************************************************************************************************/
void keyPressed(){
  /* detects key presses */
  if (key == '1'  || key == '2' || key == '3'){
    println(key);
    /* clear the world of objects */
      world.remove(l1);
      world.remove(l2);
      world.remove(l3);
      world.remove(l4);
      world.remove(wall1);
      world.remove(wall2);
      world.remove(wall3);
     if (key == '1'){
       world.add(l1);
       world.add(l2);
       scene = 0;
     } else if (key == '2'){
       world.add(l3);
     }else{
      world.add(l4);
      world.add(wall1);
      world.add(wall2);
      world.add(wall3);
     }
      
      
  }
  
  
}
/* end keypress section ************************************************************************************************/


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      lastPosEE.set(posEE.copy());
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    pantograph.setGain(1.0f);
    /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(l1)){
      dampening = max(dampening*0.9, PVector.dist(posEE, lastPosEE)*2000);
      s.h_avatar.setDamping(dampening);
    }
    else if (s.h_avatar.isTouchingBody(l3)){
      s.h_avatar.setDamping(-posEE.x *100);
    }
    else if (s.h_avatar.isTouchingBody(l4)){
      pantograph.setGain(max(1.0f,-posEE.x*2));
    }else{
      s.h_avatar.setDamping(100); 
      
    }

  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
