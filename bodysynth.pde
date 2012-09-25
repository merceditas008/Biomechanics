import SimpleOpenNI.*;
SimpleOpenNI  kinect;

//MIDI libraries
import themidibus.*; //Import the library
MidiBus myBus; // The MidiBus

boolean lefthand_UP = false;

void setup() {

  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  size(640, 480);
  fill(255, 0, 0);

  MidiBus.list(); // List all available Midi devices on STDOUT. This will show each device's index and name.
  myBus = new MidiBus(this, 0, 0); // Create a new MidiBus using the device index to select the Midi input and output devices respectively.
}

void draw() {

  kinect.update();
  image(kinect.depthImage(), 0, 0);

  IntVector userList = new IntVector();
  kinect.getUsers(userList);

  if (userList.size() > 0) {
    int userId = userList.get(0);

    if ( kinect.isTrackingSkeleton(userId)) {
      drawSkeleton(userId);
    }
  }
}

void drawSkeleton(int userId) {

  //set up the MIDI channel
  int channel = 1;

  stroke(0);
  strokeWeight(15);
  
  
    stroke(57,206,224);
    fill(57,206,224);
    kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
    kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);
    kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
    kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);
    noStroke();
    fill(255, 0, 0);
   // drawJoint(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER);
    drawJoint(userId, SimpleOpenNI.SKEL_LEFT_ELBOW);
    drawJoint(userId, SimpleOpenNI.SKEL_LEFT_HAND);
    //drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
    drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW);
    drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_HAND);
    drawJoint(userId, SimpleOpenNI.SKEL_RIGHT_HIP);
    drawJoint(userId, SimpleOpenNI.SKEL_LEFT_HIP);
    drawJoint(userId, SimpleOpenNI.SKEL_HEAD);
    
    
    float leftHandY = check_positiony_JOIN(userId,SimpleOpenNI.SKEL_LEFT_HAND);
    float rightHandY = check_positiony_JOIN(userId,SimpleOpenNI.SKEL_RIGHT_HAND); 
    
    float leftElbow = check_positiony_JOIN(userId,SimpleOpenNI.SKEL_LEFT_ELBOW);
    float rightElbow = check_positiony_JOIN(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW);
    
    
    float leftHip = check_positionx_JOIN(userId,SimpleOpenNI.SKEL_LEFT_HIP);
    float rightHip = check_positionx_JOIN(userId,SimpleOpenNI.SKEL_RIGHT_HIP);
    
    
    float head = check_positionx_JOIN(userId,SimpleOpenNI.SKEL_HEAD);


    
    //send CHANNEL 1, CONTROL NUM1
    myBus.sendControllerChange(1,1,int((leftHandY/height)*127));
     //send CHANNEL 1, CONTROL NUM2
    myBus.sendControllerChange(2,2,int((rightHandY/height)*127));
     //send CHANNEL 1, CONTROL NUM3
    myBus.sendControllerChange(3,3,int((leftElbow/height)*127));
     //send CHANNEL 1, CONTROL NUM4
    myBus.sendControllerChange(4,4,int((rightElbow/height)*127));
    
   
//    //send CHANNEL 1, CONTROL NUM5
   myBus.sendControllerChange(5,5,int((leftHip/width)*127));
//    //send CHANNEL 1, CONTROL NUM6
  myBus.sendControllerChange(6,6,int((rightHip/width)*127));
//    //send CHANNEL 1, CONTROL NUM4
  myBus.sendControllerChange(7,7,int((head/width)*127));
    
  }



void drawJoint(int userId, int jointID) {
  PVector joint = new PVector();
  float confidence = kinect.getJointPositionSkeleton(userId, jointID, joint);
  if (confidence < 0.5) {
    return;
  }
  PVector convertedJoint = new PVector();
  kinect.convertRealWorldToProjective(joint, convertedJoint);
  ellipse(convertedJoint.x, convertedJoint.y, 10, 10);
}


void check_position_LEFT_HAND(int userId, int jointID){
  
  PVector joint_check = new PVector();
  float confidence_check = kinect.getJointPositionSkeleton(userId, jointID, joint_check);
  if (confidence_check < 0.5) {
    return;
  }
  PVector convertedJoint_check = new PVector();
  kinect.convertRealWorldToProjective(joint_check, convertedJoint_check);
  if (convertedJoint_check.y <50){
    
    println("LADO IZQUIERDO!!");
    lefthand_UP = !lefthand_UP;
  }
}


float check_positiony_JOIN(int userId, int jointID) {

  PVector joint_check = new PVector();
  float confidence_check = kinect.getJointPositionSkeleton(userId, jointID, joint_check);
  if (confidence_check < 0.5) {
    return 0;
  }
  PVector convertedJoint_check = new PVector();
  kinect.convertRealWorldToProjective(joint_check, convertedJoint_check);
  return convertedJoint_check.y;
  
}

float check_positionx_JOIN(int userId, int jointID) {

  PVector joint_check = new PVector();
  float confidence_check = kinect.getJointPositionSkeleton(userId, jointID, joint_check);
  if (confidence_check < 0.5) {
    return 0;
  }
  PVector convertedJoint_check = new PVector();
  kinect.convertRealWorldToProjective(joint_check, convertedJoint_check);
  return convertedJoint_check.x;
  
}

  



float calculateAngle(int userId, int jointID_origin, int jointID_end) {

  PVector joint_origin = new PVector();
  float confidence = kinect.getJointPositionSkeleton(userId, jointID_origin, joint_origin);
  if (confidence < 0.5) {
    return 0;
  }
  PVector convertedJoint_origin = new PVector();
  kinect.convertRealWorldToProjective(joint_origin, convertedJoint_origin);
  //  println("coordenada x_origin   " + convertedJoint_origin.x);
  //  println("coordenada y_origin  " + convertedJoint_origin.y);


  PVector joint_end = new PVector();
  float confidence2 = kinect.getJointPositionSkeleton(userId, jointID_end, joint_end);
  if (confidence2 < 0.5) {
    return 0;
  }
  PVector convertedJoint_end = new PVector();
  kinect.convertRealWorldToProjective(joint_end, convertedJoint_end);
  //  println("coordenada x_end   " + convertedJoint_end.x);
  //  println("coordenada y_end   " + convertedJoint_end.y);
  //  
  float angle_rad = atan(abs(convertedJoint_end.x - convertedJoint_origin.x)/abs(convertedJoint_end.y - convertedJoint_origin.y));
  float angle_deg = (angle_rad*360 /( 2*PI));
  //println("angulo  " + angle_deg);

  return angle_deg;
}



// user-tracking callbacks!
void onNewUser(int userId) {
  println("start pose detection");
  kinect.startPoseDetection("Psi", userId);
}



void onEndCalibration(int userId, boolean successful) {
  if (successful) { 
    println("  User calibrated !!!");
    kinect.startTrackingSkeleton(userId);
  } 
  else { 
    println("  Failed to calibrate user !!!");
    kinect.startPoseDetection("Psi", userId);
  }
}



void onStartPose(String pose, int userId) {
  println("Started pose for user");
  kinect.stopPoseDetection(userId); 
  kinect.requestCalibrationSkeleton(userId, true);
}

