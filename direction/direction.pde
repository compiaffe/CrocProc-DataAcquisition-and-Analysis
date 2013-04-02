

import processing.serial.*;
int lf = 10;    // Linefeed in ASCII
float angle;
int s0, s1, s2, x;
int t_window = 30000;//window of a 3 tenth of a second
PVector[] fade_draw = new PVector[10];

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
  for (x=0;x<10;x++) {

    fade_draw[x]= new PVector();
    fade_draw[x].x = 1;
    fade_draw[x].y = 1;
  }
}

void draw() {
  PVector draw = new PVector(0, 0);

  while (myPort.available () > 0) {
    String inBuffer = myPort.readStringUntil(lf);   
    if (inBuffer != null) {
      int[] nums = int(split(trim(inBuffer), ' '));
      if (nums[0] == 0) {
        s0 = nums[1];
        if ((s0-s1<t_window) && (s0-s2<t_window)) { //if both other sensor values are from the same incident
          draw = triangulate(s0, s1, s2);
          fading_draw(draw);
        }
      }
      if (nums[0] == 1) {
        s1 = nums[1];
        if ((s1-s0<t_window) && (s1-s2<t_window)) { //if both other sensor values are from the same incident
          draw = triangulate(s0, s1, s2);
          fading_draw(draw);
        }
      }
      if (nums[0] == 2) {
        s2 = nums[1];
        if ((s2-s1<t_window) && (s2-s0<t_window)) { //if both other sensor values are from the same incident
          draw = triangulate(s0, s1, s2);
          fading_draw(draw);
        }
      }
    }
  }
}

PVector triangulate(int s0, int s1, int s2) {//triangulate and return the vector pointing 
  float rad2deg = 180/PI;
  PVector v12 = new PVector(1, 0);
  PVector v13 = new PVector(0.5, (sqrt(3)/2));
  PVector v23 = new PVector(-0.5, (sqrt(3)/2));
  PVector vector = new PVector(0, 0);
  int x12, x13, x23, x;

  x12 = s0-s1;
  x13 = s0-s2;
  x23 = s1-s2;
  v12.mult(x12);
  v13.mult(x13);
  v23.mult(x23);
  vector = PVector.add(v12, v13, v23);

  // calc the angle
  angle = atan2(vector.x, vector.y)*rad2deg;
  println(angle);

  return vector;
}

void fading_draw(PVector draw) {


  PVector center = new PVector(width/2, height/2);
  // Subtract center from mouse which results in a vector that points from center to mouse
  draw.sub(center);

  // Normalize the vector
  draw.normalize();

  // Multiply its length by 150 (Scaling its length)
  draw.mult(150);

  translate(width/2, height/2);
  // Draw the resulting vector

  background(220); 

  for (x = 9 ; x>0 ; x--) {
    fade_draw[x]=fade_draw[x-1];
  }
  fade_draw[0] = draw;
  for (x = 0; x < 10; x++) { //draw the last 10 fading out...

    stroke(0, (255/(x+1)));
    line(0, 0, fade_draw[x].x, fade_draw[x].y);
  }
}

