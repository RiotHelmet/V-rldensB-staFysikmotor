let canvas = document.getElementById("canvas");
let ctx = canvas.getContext("2d");

class Vec2 {
  //Vector class

  x;
  y;
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }

  mult(k) {
    return new Vec2(this.x * k, this.y * k);
  }

  add(vecB) {
    return new Vec2(this.x + vecB.x, this.y + vecB.y);
  }

  subtract(vecB) {
    return new Vec2(this.x - vecB.x, this.y - vecB.y);
  }

  len() {
    return Math.sqrt(this.x ** 2 + this.y ** 2);
  }

  sqrLen() {
    return this.x ** 2 + this.y ** 2;
  }

  normal() {
    return new Vec2(-this.y, this.x);
  }

  normalize() {
    if (this.x == 0 && this.y == 0) {
      return new Vec2(0, 0);
    }
    let hypo = Math.sqrt(this.x ** 2 + this.y ** 2);

    return new Vec2(this.x / hypo, this.y / hypo);
  }
}

function CrossProduct(a, b) {
  return a.x * b.y - a.y * b.x;
}

function CrossProductF_V(a, b) {
  return new Vec2(-a * b.y, a * b.x);
}

function CrossProductV_F(a, b) {
  return new Vec2(b * a.y, -b * a.x);
}

function rotationMat(rad) {
  return [
    [Math.cos(rad), -Math.sin(rad)],
    [Math.sin(rad), Math.cos(rad)],
  ];
}

function MATxVEC2(Matrix, Vector) {
  return new Vec2(
    Matrix[0][0] * Vector.x + Matrix[0][1] * Vector.y,
    Matrix[1][0] * Vector.x + Matrix[1][1] * Vector.y
  );
}

function MAT22xMAT22(MatrixA, MatrixB) {
  returnMatrix = [];
  for (let i = 0; i < MatrixA.length; i++) {
    returnMatrix.push([]);

    for (let j = 0; j < MatrixB[0].length; j++) {
      returnMatrix[i].push([]);
    }
  }

  for (let i = 0; i < MatrixA.length; i++) {
    for (let j = 0; j < MatrixB[0].length; j++) {
      returnMatrix[i][j] = 0;
      for (let k = 0; k < MatrixA[0].length; k++) {
        returnMatrix[i][j] += MatrixA[i][k] * MatrixB[k][j];
      }
    }
  }
  return returnMatrix;
}

function mirrorMatrix(Matrix) {
  let multMatrix = [];

  for (let i = 0; i < Matrix.length; i++) {
    multMatrix.push([]);
    for (let j = 0; j < Matrix.length; j++) {
      multMatrix[i].push(0);
    }
    multMatrix[i][Matrix.length - i - 1] = 1;
  }

  return MAT22xMAT22(multMatrix, Matrix);
}

function translateMat(Mat22, transVector) {
  returnMatrix = [];
  for (let i = 0; i < Mat22.length; i++) {
    returnMatrix.push([
      Mat22[i][0] + transVector.x,
      Mat22[i][1] + transVector.y,
    ]);
  }

  return returnMatrix;
}

function distanceFromPointToLineSqrd(v, w, p) {
  if (!v.x) {
    v = new Vec2(v[0], v[1]);
    w = new Vec2(w[0], w[1]);
    p = new Vec2(p[0], p[1]);
  }
  return (
    ((w.x - v.x) * (v.y - p.y) - (v.x - p.x) * (w.y - v.y)) ** 2 /
    ((w.x - v.x) ** 2 + (w.y - v.x) ** 2)
  );
}

function pDistance(x, y, x1, y1, x2, y2) {
  var A = x - x1;
  var B = y - y1;
  var C = x2 - x1;
  var D = y2 - y1;

  var dot = A * C + B * D;
  var len_sq = C * C + D * D;
  var param = -1;
  if (len_sq != 0)
    //in case of 0 length line
    param = dot / len_sq;

  var xx, yy;

  if (param < 0) {
    xx = x1;
    yy = y1;
  } else if (param > 1) {
    xx = x2;
    yy = y2;
  } else {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  var dx = x - xx;
  var dy = y - yy;
  return [dx * dx + dy * dy, new Vec2(xx, yy)];
}

function nearlyEqual(a, b) {
  return Math.abs(b - a) < 0.005;
}

function drawLine(origin, dir, len, color, width) {
  ctx.strokeStyle = "black";
  ctx.lineWidth = 1;
  if (color) {
    ctx.strokeStyle = color;
  }

  if (width) {
    ctx.lineWidth = width;
  }

  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(origin.x + dir.x * len, origin.y + dir.y * len);
  ctx.stroke();
}
