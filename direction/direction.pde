
import processing.serial.*;
int lf = 10;    // Linefeed in ASCII
float angle;
int s0, s1, s2, x;
int t_window = 30000;//window of a 3 tenth of a second
float[] fade_draw = new float[20];
float draw = 0;
float rad2deg = 180/PI;



Serial myPort;  // The serial port


void setup() {
  // List all the available serial ports:
  println(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, "/dev/tty.usbserial-A900abKE", 57600);

  size(640, 360);
  smooth();
  background(0);


  //setup the fading drawing
  for (x=0;x<fade_draw.length;x++) {

    fade_draw[x]= 0;
  }
}

void draw() {

  while (myPort.available () > 0) {
    String inBuffer = myPort.readStringUntil(lf);   
    if (inBuffer != null) {
      int[] nums = int(split(trim(inBuffer), ' '));
      if (nums[0] == 0) {
        s1 = nums[1];
        if ((s0-s1<t_window) && (s0-s2<t_window)) { //if both other sensor values are from the same incident
          draw = triangulate(s0, s1, s2);
          //println("s1:"+ draw*rad2deg);
          nums[0]= 3;
        }
      }
      else if (nums[0] == 1) {
        s0 = nums[1];
        if ((s1-s0<t_window) && (s1-s2<t_window)) { //if both other sensor values are from the same incident
          draw = triangulate(s0, s1, s2);
          nums[0]= 3;
          //println("s2:"+ draw*rad2deg);
        }
      }
      else if (nums[0] == 2) {
        s2 = nums[1];
        if ((s2-s1<t_window) && (s2-s0<t_window)) { //if both other sensor values are from the same incident
          draw = triangulate(s0, s1, s2);
          nums[0]= 3;
          //println("s3:"+draw*rad2deg);
        }
      }

      fading_draw(draw);
    }
    else {
      fading_draw(draw);
    }
  }
}

float triangulate(int s0, int s1, int s2) {//triangulate and return the vector pointing 
  PVector v12 = new PVector(1, 0);
  PVector v13 = new PVector(0.5, (sqrt(3)/2));
  PVector v23 = new PVector(-0.5, (sqrt(3)/2));
  PVector vector = new PVector(0, 0);
  int x12, x13, x23, x;
  float angle;

  x12 = s0-s1;
  x13 = s0-s2;
  x23 = s1-s2;
  v12.mult(x12);
  v13.mult(x13);
  v23.mult(x23);
  vector = PVector.add(v12, v13, v23);
  // calc the angle
  angle = atan2(vector.x, vector.y);

  return angle;
}

void fading_draw(float angle) {
  background(220); 

  for (x = fade_draw.length-1 ; x>0 ; x--) {
    fade_draw[x]=fade_draw[x-1];
  }
  fade_draw[0] = angle;
  background(220); 

  for (x = 0; x < fade_draw.length; x++) { //draw the last 10 fading out...
    // Draw the resulting vector
    pushMatrix();
    translate(width/2, height/2 );
    rotate(fade_draw[x]);
    stroke(0, (255/(x+1))); 
    line(0, 0, 150, 0);
    popMatrix();
  }
}

