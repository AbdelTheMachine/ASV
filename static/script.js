const socket = io();
const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");

function setMode(mode) {
  fetch("/toggle_mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode })
  });
}

// Convert world coordinates to canvas coordinates
function worldToCanvas(x, y, centerX, centerY, scale = 2) {
  const canvasX = canvas.width / 2 + (x - centerX) * scale;
  const canvasY = canvas.height / 2 - (y - centerY) * scale;
  return { x: canvasX, y: canvasY };
}

socket.on("map_update", (data) => {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const scale = 2;
  const centerX = data.robot.x;
  const centerY = data.robot.y;

  // Draw walls
  ctx.fillStyle = "red";
  data.walls.forEach(wall => {
    const p = worldToCanvas(wall.x, wall.y, centerX, centerY, scale);
    ctx.fillRect(p.x - 2, p.y - 2, 4, 4);
  });

  // Draw robot
  const r = worldToCanvas(data.robot.x, data.robot.y, centerX, centerY, scale);
  ctx.fillStyle = "blue";
  ctx.beginPath();
  ctx.arc(r.x, r.y, 6, 0, 2 * Math.PI);
  ctx.fill();

  // Draw heading
  const hx = r.x + 10 * Math.cos(data.robot.heading);
  const hy = r.y - 10 * Math.sin(data.robot.heading);
  ctx.strokeStyle = "blue";
  ctx.beginPath();
  ctx.moveTo(r.x, r.y);
  ctx.lineTo(hx, hy);
  ctx.stroke();

  // Draw orange pins
  ctx.fillStyle = "orange";
  if (data.pins) {
    data.pins.forEach(pin => {
      const p = worldToCanvas(pin.x, pin.y, centerX, centerY, scale);
      ctx.fillRect(p.x - 3, p.y - 3, 6, 6);
    });
  }
});
