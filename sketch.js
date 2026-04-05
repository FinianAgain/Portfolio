function setup() {
  frameRate(60);
  colorMode('HSB');
  cnv = createCanvas(windowWidth, windowHeight);
  cnv.parent("sketch-container")
  cnv.style('position', 'fixed');
  cnv.style('top', '0');
  cnv.style('left', '0');


  let num_links = 5;
  let links = [];
  let start_angles = [];
  for (let i=1; i <= num_links; i++) {
    links.push(width / (i * 3));
    start_angles.push(0);
  }
    test = new Linkage(
      createVector(width, height/2), 
      links, 
      start_angles); 
}

function draw() {
  clear();
  test.render();
  test.linear_interp();
}

function mouseClicked() {
  let set_point = createVector(mouseX, mouseY);
  test.update(set_point);
}

function touchMoved() {
  let set_point = createVector(touch.x, touch.y);
  test.update(set_point);
}

function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
}
