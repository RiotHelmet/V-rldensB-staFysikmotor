let canvas = document.getElementById("canvas");
let ctx = canvas.getContext("2d");

objects = [];
bodyCount = 0;

physicsQueue = [];

EPSILON = 0.0005;

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

class mass_data {
  mass;
  inv_mass;

  inertia = 5;
  inverse_inertia = 0.01;

  constructor(_mass) {
    this.mass = _mass;
    if (this.mass == 0) {
      this.inv_mass = 0;
    } else {
      this.inv_mass = 1 / _mass;
    }
  }
}

class material_data {
  density;
  restitution;

  constructor(_material) {
    switch (_material) {
      case "Rock":
        this.density = 0.6;
        this.restitution = 0.1;
        break;

      case "Wood":
        this.density = 0.3;
        this.restitution = 0.2;
        break;

      case "Metal":
        this.density = 1.2;
        this.restitution = 0.05;
        break;

      case "Static":
        this.density = 0.0;
        this.restitution = 0.4;
        break;

      default:
        this.density = 0.6;
        this.restitution = 0.1;
        break;
    }
  }
}

class transform_data {
  position = new Vec2();
  rotation = 0;
}

class body {
  id;
  shape;
  transform = new transform_data();
  material;
  mass_data;
  linearVelocity = new Vec2();
  force = new Vec2(0, 0);

  angularVelocity = 0;
  torque = 0;

  gravityScale;

  constructor(_shape, _x, _y, _material, _mass) {
    this.id = bodyCount;
    bodyCount++;

    this.shape = _shape;
    this.transform.position.x = _x;
    this.transform.position.y = _y;
    this.material = new material_data(_material);
    this.mass_data = new mass_data(_mass);
    this.linearVelocity = new Vec2(0, 0);

    objects.push(this);
  }

  update() {
    if (Math.abs(this.force.x) < EPSILON) {
      this.force.x = 0;
    }

    if (Math.abs(this.force.y) < EPSILON) {
      this.force.y = 0;
    }

    if (Math.abs(this.linearVelocity.x) < EPSILON) {
      this.linearVelocity.x = 0;
    }

    if (Math.abs(this.linearVelocity.y) < EPSILON) {
      this.linearVelocity.y = 0;
    }

    this.linearVelocity = this.linearVelocity.add(
      this.force.mult(this.mass_data.inv_mass * dt)
    );

    this.angularVelocity += this.torque * this.mass_data.inverse_inertia * dt;

    this.transform.position = this.transform.position.add(
      this.linearVelocity.mult(dt)
    );

    this.transform.rotation += this.angularVelocity * dt;

    this.force = new Vec2(0, 0);
  }

  draw() {
    this.shape.draw(this.transform.position, this.transform.rotation);
  }
}

class Circle {
  type = "Circle";
  radius;
  frictionCoefficientStatic = 0.001;
  frictionCoefficientDynamic = 0.001;
  constructor(_radius) {
    this.radius = _radius;
  }
  draw(_position, rotation) {
    ctx.strokeStyle = "black";
    ctx.beginPath();
    ctx.arc(_position.x, _position.y, this.radius, 0, 2 * Math.PI);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(_position.x, _position.y);
    ctx.lineTo(
      _position.x + Math.cos(rotation) * this.radius,
      _position.y + Math.sin(rotation) * this.radius
    );
    ctx.stroke();
  }
}

class OBB {
  type = "OBB";
  min = new Vec2();
  max = new Vec2();
  frictionCoefficientStatic = 0.6;
  frictionCoefficientDynamic = 0.4;

  size = { x: 0, y: 0 };

  vertices;

  constructor(sizeX, sizeY) {
    this.size.x = sizeX;
    this.size.y = sizeY;

    this.vertices = [
      [-this.size.x, -this.size.y],
      [this.size.x, -this.size.y],
      [this.size.x, this.size.y],
      [-this.size.x, this.size.y],
    ];
  }

  draw(_position, _rotation) {
    let translatedVertices = translateMat(
      MAT22xMAT22(this.vertices, rotationMat(_rotation)),
      _position
    );

    ctx.strokeStyle = "black";
    ctx.beginPath();
    ctx.moveTo(translatedVertices[0][0], translatedVertices[0][1]);
    ctx.lineTo(translatedVertices[1][0], translatedVertices[1][1]);
    ctx.lineTo(translatedVertices[2][0], translatedVertices[2][1]);
    ctx.lineTo(translatedVertices[3][0], translatedVertices[3][1]);
    ctx.lineTo(translatedVertices[0][0], translatedVertices[0][1]);
    ctx.stroke();
  }
}

// function calculateMass(Body) {

// }

function CirclevsCircle(_M) {
  let _A = _M.A;
  let _B = _M.B;

  normal = new Vec2(
    _B.transform.position.x - _A.transform.position.x,
    _B.transform.position.y - _A.transform.position.y
  );

  normalLengthSqrd = normal.x ** 2 + normal.y ** 2;

  totalRadius = _A.shape.radius + _B.shape.radius;

  if (totalRadius ** 2 < normalLengthSqrd) {
    return false;
  }

  normalLength = Math.sqrt(normalLengthSqrd);

  M.penetrationDepth = totalRadius - normalLength;

  M.normal = normal.normalize();

  ctx.strokeStyle = "red";

  ctx.beginPath();
  ctx.moveTo(
    M.normal.x * _A.shape.radius + _A.transform.position.x,
    M.normal.y * _A.shape.radius + _A.transform.position.y
  );
  ctx.lineTo(
    M.normal.x * (_A.shape.radius + 8) + _A.transform.position.x,
    M.normal.y * (_A.shape.radius + 8) + _A.transform.position.y
  );
  ctx.stroke();

  return true;
}

function AABBvsCircle(_M) {
  let _A = _M.A;
  let _B = _M.B;

  normal = new Vec2(
    _B.transform.position.x - _A.transform.position.x,
    _B.transform.position.y - _A.transform.position.y
  );

  closest = new Vec2(normal.x, normal.y);

  x_extent = (_A.shape.max.x - _A.shape.min.x) / 2;
  y_extent = (_A.shape.max.y - _A.shape.min.y) / 2;

  closest.x = Math.max(-x_extent, Math.min(x_extent, closest.x));
  closest.y = Math.max(-y_extent, Math.min(y_extent, closest.y));

  inside = false;

  if (normal.x == closest.x && normal.y == closest.y) {
    inside = true;
    if (Math.abs(normal.x) > Math.abs(normal.y)) {
      if (closest.x > 0) {
        closest.x = x_extent;
      } else {
        closest.x = -x_extent;
      }
    } else {
      if (closest.y > 0) {
        closest.y = y_extent;
      } else {
        closest.y = -y_extent;
      }
    }
  }

  normal = new Vec2(normal.x - closest.x, normal.y - closest.y);
  // ctx.beginPath();
  // ctx.moveTo(
  //   normal.x * _A.shape.radius + _A.transform.position.x,
  //   normal.y * _A.shape.radius + _A.transform.position.y
  // );
  // ctx.lineTo(
  //   normal.x * (_A.shape.radius + 8) + _A.transform.position.x,
  //   normal.y * (_A.shape.radius + 8) + _A.transform.position.y
  // );
  // ctx.stroke();

  distance = normal.x ** 2 + normal.y ** 2;

  if (distance > _B.shape.radius ** 2 && !inside) {
    return false;
  }

  distance = Math.sqrt(distance);

  if (inside) {
    M.normal = -normal.normalize();
    M.penetrationDepth = _B.shape.radius - distance;
  } else {
    M.normal = normal.normalize();
    M.penetrationDepth = _B.shape.radius - distance;
  }
  return true;
}

function GetSupport(vertices, directionVec) {
  bestProjection = -Infinity;
  let bestVertex;

  for (let i = 0; i < vertices.length; i++) {
    const v = new Vec2(vertices[i][0], vertices[i][1]);
    projection = DotProduct(v, directionVec);

    if (projection > bestProjection) {
      bestVertex = v;
      bestProjection = projection;
    }
  }
  return bestVertex;
}

function arrayContainVector(Array, object) {
  for (let i = 0; i < Array.length; i++) {
    if (object.x == -0) {
      object.x == 0;
    }
    if (object.y == -0) {
      object.y == 0;
    }

    if (Array[i].x == object.x && Array[i].y == object.y) {
      return true;
    }
  }
  return false;
}

function SAT_recieveNormals(A_translatedVertices, B_translatedVertices) {
  let normals = [new Vec2(1, 0), new Vec2(0, 1)];

  for (let i = 1; i < A_translatedVertices.length; i++) {
    let axis = new Vec2(
      A_translatedVertices[(i + 1) % A_translatedVertices.length][0] -
        A_translatedVertices[i][0],
      A_translatedVertices[(i + 1) % A_translatedVertices.length][1] -
        A_translatedVertices[i][1]
    )
      .normalize()
      .normal();
    if (
      !arrayContainVector(normals, axis) &&
      !arrayContainVector(normals, axis.mult(-1))
    ) {
      normals.push(axis);
    }
  }

  for (let i = 1; i < B_translatedVertices.length; i++) {
    let axis = new Vec2(
      B_translatedVertices[(i + 1) % B_translatedVertices.length][0] -
        B_translatedVertices[i][0],
      B_translatedVertices[(i + 1) % B_translatedVertices.length][1] -
        B_translatedVertices[i][1]
    )
      .normalize()
      .normal();
    if (
      !arrayContainVector(normals, axis) &&
      !arrayContainVector(normals, axis.mult(-1))
    ) {
      normals.push(axis);
    }
  }
  return normals;
}

function SAT(_M) {
  let _A = _M.A;
  let _B = _M.B;

  normal = _B.transform.position.subtract(_A.transform.position);

  let A_translatedVertices = translateMat(
    MAT22xMAT22(_A.shape.vertices, rotationMat(_A.transform.rotation)),
    _A.transform.position
  );

  let B_translatedVertices = translateMat(
    MAT22xMAT22(_B.shape.vertices, rotationMat(_B.transform.rotation)),
    _B.transform.position
  );

  axisList = SAT_recieveNormals(A_translatedVertices, B_translatedVertices);

  let overlapList = [];

  for (let i = 0; i < axisList.length; i++) {
    const axis = axisList[i];

    a_extent =
      (DotProduct(GetSupport(A_translatedVertices, axis), axis) -
        DotProduct(GetSupport(A_translatedVertices, axis.mult(-1)), axis)) /
      2;

    b_extent =
      (DotProduct(GetSupport(B_translatedVertices, axis), axis) -
        DotProduct(GetSupport(B_translatedVertices, axis.mult(-1)), axis)) /
      2;

    overlap = b_extent + a_extent - Math.abs(DotProduct(normal, axis));

    if (overlap < 0) {
      return false;
    }
    overlapList.push(overlap);
  }

  M.penetrationDepth = Math.min(...overlapList);

  axis = axisList[overlapList.indexOf(M.penetrationDepth)];

  if (DotProduct(normal, axis) < 0) {
    M.normal = axis.mult(-1);
  } else {
    M.normal = axis;
  }

  return true;
}

function distanceFromPointToLineSqrd(v, w, p) {
  v = new Vec2(v[0], v[1]);
  w = new Vec2(w[0], w[1]);
  p = new Vec2(p[0], p[1]);

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
  return dx * dx + dy * dy;
}

// function nearlyEqual(a, b) {
//   return Math.abs(b - a) < 1;
// }

function findContactPoints(_M) {
  // Only runs if collision

  _A = _M.A;
  _B = _M.B;

  let A_translatedVertices = translateMat(
    MAT22xMAT22(_A.shape.vertices, rotationMat(_A.transform.rotation)),
    _A.transform.position
  );

  let B_translatedVertices = translateMat(
    MAT22xMAT22(_B.shape.vertices, rotationMat(_B.transform.rotation)),
    _B.transform.position
  );

  let closestDist = Infinity;

  let contact1;
  let contact2;
  let pointCount;

  for (let i = 0; i < A_translatedVertices.length; i++) {
    point = A_translatedVertices[i];

    for (let j = 0; j < B_translatedVertices.length; j++) {
      edgePointA = B_translatedVertices[j];
      edgePointB = B_translatedVertices[(j + 1) % B_translatedVertices.length];

      distance = pDistance(
        point[0],
        point[1],
        edgePointA[0],
        edgePointA[1],
        edgePointB[0],
        edgePointB[1]
      );

      if (Math.abs(distance - closestDist) < 0.01) {
        if (point[0] != contact1[0] && point[1] != contact1[1]) {
          contact2 = point;
          pointCount = 2;
        }
      } else if (distance < closestDist) {
        closestDist = distance;
        contact1 = point;
        pointCount = 1;
      }
    }
  }

  for (let i = 0; i < B_translatedVertices.length; i++) {
    point = B_translatedVertices[i];

    for (let j = 0; j < A_translatedVertices.length; j++) {
      edgePointA = A_translatedVertices[j];
      edgePointB = A_translatedVertices[(j + 1) % A_translatedVertices.length];

      distance = pDistance(
        point[0],
        point[1],
        edgePointA[0],
        edgePointA[1],
        edgePointB[0],
        edgePointB[1]
      );

      if (Math.abs(distance - closestDist) < 0.01) {
        if (point[0] != contact1[0] && point[1] != contact1[1]) {
          contact2 = point;
          pointCount = 2;
        }
      } else if (distance < closestDist) {
        closestDist = distance;
        contact1 = point;
        pointCount = 1;
      }
    }
  }

  ctx.strokeStyle = "red";

  if (pointCount == 1) {
    ctx.beginPath();
    ctx.moveTo(contact1[0], contact1[1]);
    ctx.lineTo(contact1[0] + _M.normal.x * 10, contact1[1] + _M.normal.y * 10);
    ctx.stroke();
    contact1 = new Vec2(contact1[0], contact1[1]);
    _M.contactPoints = [contact1];
    _M.contactCount = 1;

    return;
  } else if (pointCount == 2) {
    ctx.beginPath();
    ctx.moveTo(contact1[0], contact1[1]);
    ctx.lineTo(contact1[0] + _M.normal.x * 10, contact1[1] + _M.normal.y * 10);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(contact2[0], contact2[1]);
    ctx.lineTo(contact2[0] + _M.normal.x * 10, contact2[1] + _M.normal.y * 10);
    ctx.stroke();
    contact1 = new Vec2(contact1[0], contact1[1]);
    contact2 = new Vec2(contact2[0], contact2[1]);

    _M.contactPoints = [contact1, contact2];
    _M.contactCount = 2;

    return;
  }
}

function DotProduct(a, b) {
  return a.x * b.x + a.y * b.y;
}

function collisionResolve(_M) {
  let _A = _M.A;
  let _B = _M.B;

  PositionalCorrection(_M);

  findContactPoints(_M);

  let normal = _M.normal;

  // console.log(_M);

  for (let i = 0; i < _M.contactCount; i++) {
    const contactPoint = _M.contactPoints[i];
    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(contactPoint.x, contactPoint.y, 2, 0, Math.PI * 2);
    ctx.fill();
  }

  e = Math.min(_A.material.restitution, _B.material.restitution);

  for (let i = 0; i < _M.contactCount; i++) {
    const contactPoint = _M.contactPoints[i];

    rAP = contactPoint.subtract(_A.transform.position);

    rBP = contactPoint.subtract(_B.transform.position);

    rAPerp = new Vec2(rAP.y, -rAP.x);

    rBPerp = new Vec2(rBP.y, -rBP.x);

    vAP = _A.linearVelocity.add(rAPerp.mult(_A.angularVelocity));

    vBP = _B.linearVelocity.add(rBPerp.mult(_B.angularVelocity));

    let relativeVelocity = vBP.subtract(vAP);

    if (relativeVelocity.len() < new Vec2(0, 100).mult(dt).len() + EPSILON) {
      e = 0;
    }

    let velocityAlongNormal = DotProduct(relativeVelocity, normal);

    if (velocityAlongNormal > 0) {
      return;
    }

    j = -(1 + e) * velocityAlongNormal;

    j /=
      _A.mass_data.inv_mass +
      _B.mass_data.inv_mass +
      DotProduct(rAPerp, normal) ** 2 * _A.mass_data.inverse_inertia +
      DotProduct(rBPerp, normal) ** 2 * _B.mass_data.inverse_inertia;

    j = j / _M.contactCount;

    impulse = normal.mult(j);

    applyImpulse(_B, impulse, rBPerp);
    applyImpulse(_A, impulse.mult(-1), rAPerp);

    vAP = _A.linearVelocity.add(rAPerp.mult(_A.angularVelocity));

    vBP = _B.linearVelocity.add(rBPerp.mult(_B.angularVelocity));

    relativeVelocity = vBP.subtract(vAP);

    t = relativeVelocity.subtract(
      normal.mult(DotProduct(relativeVelocity, normal))
    );

    t = t.normalize();

    jt = -DotProduct(relativeVelocity, t);

    jt /=
      _A.mass_data.inv_mass +
      _B.mass_data.inv_mass +
      DotProduct(rAPerp, t) ** 2 * _A.mass_data.inverse_inertia +
      DotProduct(rBPerp, t) ** 2 * _B.mass_data.inverse_inertia;

    jt = jt / _M.contactCount;

    sf =
      (_A.shape.frictionCoefficientStatic +
        _B.shape.frictionCoefficientStatic) /
      2;

    df =
      (_A.shape.frictionCoefficientDynamic +
        _B.shape.frictionCoefficientDynamic) /
      2;

    console.log(Math.abs(jt) < j);
    console.log(0);

    if (Math.abs(jt) < j * sf) {
      tangentImpulse = t.mult(jt);
    } else {
      tangentImpulse = t.mult(-j * df);
    }

    applyImpulse(_B, tangentImpulse, rBPerp);
    applyImpulse(_A, tangentImpulse.mult(-1), rAPerp);
  }
}

function applyImpulse(Object, impulse, contactVector) {
  Object.linearVelocity = Object.linearVelocity.add(
    impulse.mult(Object.mass_data.inv_mass)
  );

  Object.angularVelocity +=
    DotProduct(contactVector, impulse) * Object.mass_data.inverse_inertia;
}

function PositionalCorrection(_M) {
  a = _M.A;
  b = _M.B;
  normal = _M.normal;
  penetrationDepth = _M.penetrationDepth;

  percent = 0.8; // usually 20% to 80%
  buffer = 0.05;

  correction = normal.mult(
    (Math.max(penetrationDepth - buffer, 0) /
      (a.mass_data.inv_mass + b.mass_data.inv_mass)) *
      percent
  );

  // a.force = a.force.add(normal.mult(-DotProduct(normal, a.force)));

  // b.force = b.force.add(normal.mult(-DotProduct(normal, b.force)));

  a.transform.position = a.transform.position.subtract(
    correction.mult(a.mass_data.inv_mass)
  );

  b.transform.position = b.transform.position.add(
    correction.mult(b.mass_data.inv_mass)
  );
}

class Manifold {
  A;
  B;
  penetrationDepth;
  normal;
  contactCount;
  contactPoints;

  constructor(_A, _B) {
    this.A = _A;
    this.B = _B;
  }

  update() {
    if (this.A.mass == 0 && this.B.mass == 0) {
      return;
    }
    // if (this.A.shape.type == "AABB" && this.B.shape.type == "AABB") {
    //   if (AABBvsAABB(this)) {
    //     ResolveCollision(this);
    //   }
    // }

    // if (this.A.shape.type == "Circle" && this.B.shape.type == "Circle") {

    if (SAT(this)) {
      collisionResolve(this);
    }
    // }

    if (this.A.shape.type == "Circle" && this.B.shape.type == "Circle") {
      if (CirclevsCircle(this)) {
        ResolveCollision(this);
      }
    }

    if (this.A.shape.type == "AABB" && this.B.shape.type == "Circle") {
      if (AABBvsCircle(this)) {
        ResolveCollision(this);
      }
    }
  }
}

A = new body(new OBB(1000, 10), 500, 580, "Static", 0);
A.mass_data.inertia = 0;
A.mass_data.inverse_inertia = 0;

new body(new OBB(150, 10), 300, 200, "Static", 0);
objects[objects.length - 1].mass_data.inertia = 0;
objects[objects.length - 1].mass_data.inverse_inertia = 0;
objects[objects.length - 1].transform.rotation = -0.4;

new body(new OBB(150, 10), 550, 300, "Static", 0);
objects[objects.length - 1].mass_data.inertia = 0;
objects[objects.length - 1].mass_data.inverse_inertia = 0;
objects[objects.length - 1].transform.rotation = 0.4;

// new body(new OBB(20, 20), 600, 400, "Rock", 1);

// A = new body(new OBB(20, 20), 300, 300, "Rock", 1);
A.transform.rotation = 0;

// B = new body(new OBB(20, 20), 500, 500, "Rock", 1);
// B.transform.rotation = 1;

mousePos = { x: 0, y: 0 };

document.addEventListener("mousemove", function (e) {
  mousePos.x = e.offsetX;
  mousePos.y = e.offsetY;
  // B.transform.position = new Vec2(mousePos.x, mousePos.y);
});

document.addEventListener("mousedown", function (e) {
  // B.linearVelocity.x += 10;

  new body(
    new OBB(Math.random() * 15 + 5, Math.random() * 15 + 5),
    mousePos.x,
    mousePos.y,
    "Rock",
    1
  );
  objects[objects.length - 1].transform.rotation = Math.random() * 2 * Math.PI;
});

function clear() {
  ctx.fillStyle = "white";
  ctx.beginPath();
  ctx.rect(0, 0, 1000, 600);
  ctx.fill();
}
ctx.strokeStyle = "black";

oldTime = Date.now();

function checkIfQueueContainsObjects(Queue, Objects) {
  for (let i = 0; i < Queue.length; i++) {
    const QueueObjs = Queue[i];

    if (QueueObjs[0].id == Objects[0].id && QueueObjs[1].id == Objects[1].id) {
      return true;
    }
    if (QueueObjs[0].id == Objects[1].id && QueueObjs[1].id == Objects[0].id) {
      return true;
    }
  }

  return false;
}
function gameLoop() {
  newTime = Date.now() / 1000;
  dt = newTime - oldTime;
  dt = 1;
  oldTime = Date.now() / 1000;
  clear();

  objects.forEach((Object) => {
    Object.update();
    Object.draw();

    Object.force.y += 0.003;

    objects.forEach((other) => {
      if (
        Object !== other &&
        !checkIfQueueContainsObjects(physicsQueue, [Object, other])
      ) {
        physicsQueue.push([Object, other]);
      }
    });
  });

  // M = new Manifold(A, B);
  // M.update();

  physicsQueue.forEach((Queue) => {
    M = new Manifold(Queue[0], Queue[1]);
    M.update();
  });

  physicsQueue = [];

  requestAnimationFrame(gameLoop);
}

function RenderGame() {
  clear();
  objects.forEach((Object) => {
    Object.draw();
  });
}

function UpdatePhysics(deltaT) {
  UpdatePhysics();
  RenderGame();
}

gameLoop();
