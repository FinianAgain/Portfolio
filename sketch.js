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
