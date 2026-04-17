
let set_point;

function setup() {
  frameRate(60);
  colorMode('HSB');
  cnv = createCanvas(windowWidth, windowHeight);
  cnv.parent("sketch-container")
  cnv.style('position', 'fixed');
  cnv.style('top', '0');
  cnv.style('left', '0');


  let num_links = 4;
  let links = [];
  let start_angles = [];
  for (let i=1; i <= num_links; i++) {
    links.push((width + 400 * i) / (i * 4));
    start_angles.push(0);
  }
    test = new Linkage(
      createVector((width), height/2), 
      links, 
      start_angles); 

  let ik_button = createA("Inverse Kinematics 1.1.3.pdf", "n-DOF Inverse Kinematics", '_blank');
  ik_button.parent('p5-button');
  ik_button.class('button-link');
  ik_button.position(0, 0, 'relative');
  ik_button.mouseOver(() => set_point = createVector(mouseX, mouseY));
  
  let ac_button = createA("https://www.youtube.com/watch?v=dQw4w9WgXcQ", "Passive Gimbal for Aquaculure Monitoring - COMING SOON", '_blank');
  ac_button.parent('p5-button');
  ac_button.class('button-link');
  ac_button.position(0, 0, 'relative');
  ac_button.mouseOver(() => set_point = createVector(mouseX, mouseY));

  let FEM_button = createA("https://www.youtube.com/watch?v=dQw4w9WgXcQ", "Finite Element Analysis - COMING SOON", '_blank');
  FEM_button.parent('p5-button');
  FEM_button.class('button-link');
  FEM_button.position(0, 0, 'relative');
  FEM_button.mouseOver(() => set_point = createVector(mouseX, mouseY));
}

function draw() {
  clear();
  test.render();
  test.update(set_point);
  test.linear_interp();
}

function mousePressed() {
  set_point = createVector(mouseX, mouseY);
}

function touchStarted() {
  for (let touch of touches) {
    set_point = createVector(touch.x, touch.y);
  }
} 

function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
}
