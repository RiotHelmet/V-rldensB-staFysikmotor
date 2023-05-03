objects = [];
bodyCount = 0;

currentType = "Circle";

physicsQueue = [];

EPSILON = 0.0;

class mass_data {
  mass;
  inv_mass;

  inertia = 5;
  inverse_inertia = 0.01;

  constructor() {}
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

class rigidbody {
  id;
  shape;
  transform = new transform_data();

  material;
  mass_data;
  linearVelocity = new Vec2();
  force = new Vec2(0, 0);

  type;

  angularVelocity = 0;
  torque = 0;

  gravityScale;

  constructor(_shape, _x, _y, _material, type, parent) {
    this.id = bodyCount;
    bodyCount++;

    if (this.parent) {
      this.parent = parent;
    }

    this.type = type;

    this.shape = _shape;
    this.transform.position.x = _x;
    this.transform.position.y = _y;
    this.material = new material_data(_material);

    this.mass_data = new mass_data();
    calculateMass_data(this);
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
      this.force.mult(this.mass_data.inv_mass * 1 * dt)
    );

    this.angularVelocity += this.torque * this.mass_data.inverse_inertia * dt;

    this.transform.position = this.transform.position.add(
      this.linearVelocity.mult(dt)
    );

    if (this.shape.type == "Circle") {
      this.transform.rotation += this.angularVelocity * dt * -1;
    } else {
      this.transform.rotation += this.angularVelocity * dt;
    }

    this.force = new Vec2(0, 0);
  }

  draw() {
    this.shape.draw(this.transform.position, this.transform.rotation);
  }
}

class Circle {
  type = "Circle";
  radius;
  frictionCoefficientStatic = 0.1;
  frictionCoefficientDynamic = 0.1;
  constructor(_radius) {
    this.radius = _radius;
  }
  draw(_position, rotation) {
    ctx.strokeStyle = "#6f3d2d";
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

function scaleMatrix(K, Matrix) {
  for (let i = 0; i < Matrix.length; i++) {
    for (let j = 0; j < Matrix[i].length; j++) {
      Matrix[i][j] *= K;
    }
  }
}

class OBB {
  type = "OBB";
  min = new Vec2();
  max = new Vec2();
  frictionCoefficientStatic = 0.9;
  frictionCoefficientDynamic = 0.6;

  size = { x: 0, y: 0 };

  vertices;

  constructor(vertices, scaling) {
    {
      this.vertices = vertices;
      scaleMatrix(scaling, this.vertices);

      this.vertices = mirrorMatrix(this.vertices);
    }
  }

  draw(_position, _rotation) {
    let translatedVertices = translateMat(
      MAT22xMAT22(this.vertices, rotationMat(_rotation)),
      _position
    );

    ctx.strokeStyle = "#6f3d2d";
    ctx.beginPath();
    for (let i = -1; i < translatedVertices.length; i++) {
      ctx.lineTo(
        translatedVertices[(i + 1) % translatedVertices.length][0],
        translatedVertices[(i + 1) % translatedVertices.length][1]
      );
    }
    ctx.stroke();
  }
}

function breakObject(Body) {
  let vertices = (Body.shape.vertices, rotationMat(Body.transform.rotation));

  console.log(vertices);

  for (let i = 0; i < vertices.length; i++) {
    vertex_A = new Vec2(vertices[i][0], vertices[i][1]);
    vertex_B = new Vec2(
      vertices[(i + 1) % vertices.length][0],
      vertices[(i + 1) % vertices.length][1]
    );

    COM = new Vec2(0, 0);

    position = new Vec2(
      (vertex_A.x + vertex_B.x + COM.x) / 3,
      (vertex_A.y + vertex_B.y + COM.y) / 3
    );

    new rigidbody(
      new OBB(
        [
          [vertex_A.x, vertex_A.y],
          [vertex_B.x, vertex_B.y],
          [COM.x, COM.y],
        ],
        15
      ),
      position.x + Body.transform.position.x,
      position.y + Body.transform.position.y,
      "Rock",
      "box"
    );
  }
}

function calculateMass_data(Body) {
  if (Body.shape.type == "Circle") {
    Body.mass_data.mass = Math.PI * Body.shape.radius ** 2;
    Body.mass_data.mass /= 10 ** 2;
    Body.mass_data.inertia = Body.mass_data.mass * Body.shape.radius ** 2;

    if (Body.mass_data.mass == 0) {
      Body.mass_data.inv_mass = 0;
    } else {
      Body.mass_data.inv_mass = 1 / Body.mass_data.mass;
    }

    if (Body.mass_data.inertia == 0) {
      Body.mass_data.inverse_inertia = 0;
    } else {
      Body.mass_data.inverse_inertia = 1 / Body.mass_data.inertia;
    }
  }
  if (Body.shape.type == "OBB") {
    vertices = Body.shape.vertices;
    COM = new Vec2(0, 0);
    let total_area = 0;
    let inertia = 0;
    let mass = 0;
    for (let i = 0; i < vertices.length; i++) {
      vertex_A = new Vec2(vertices[i][0], vertices[i][1]);
      vertex_B = new Vec2(
        vertices[(i + 1) % vertices.length][0],
        vertices[(i + 1) % vertices.length][1]
      );
      triangle_area = 0.5 * Math.abs(CrossProduct(vertex_A, vertex_B));
      mass_triangle = (Body.material.density * triangle_area) / 10 ** 2;

      inertia_triangle =
        (mass_triangle *
          (vertex_A.sqrLen() +
            vertex_B.sqrLen() +
            DotProduct(vertex_A, vertex_B))) /
        6;
      total_area += triangle_area;
      mass += mass_triangle;
      inertia += inertia_triangle;
    }

    // Find center of mass

    for (let i = 0; i < vertices.length; i++) {
      vertex_A = new Vec2(vertices[i][0], vertices[i][1]);
      vertex_B = new Vec2(
        vertices[(i + 1) % vertices.length][0],
        vertices[(i + 1) % vertices.length][1]
      );
      triangle_area = 0.5 * Math.abs(CrossProduct(vertex_A, vertex_B));
      COM = COM.add(
        vertex_A.add(vertex_B).mult(triangle_area / (3 * total_area))
      );
    }

    Body.shape.vertices = translateMat(vertices, COM.mult(-1));

    Body.mass_data.mass = mass;
    if (Body.mass_data.mass == 0) {
      Body.mass_data.inv_mass = 0;
    } else {
      Body.mass_data.inv_mass = 1 / mass;
    }

    Body.mass_data.inertia = inertia;

    if (Body.mass_data.inertia == 0) {
      Body.mass_data.inverse_inertia = 0;
    } else {
      Body.mass_data.inverse_inertia = 1 / Body.mass_data.inertia;
    }
  }
}

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

  _M.penetrationDepth = totalRadius - normalLength;

  _M.normal = normal.normalize();

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

    if (
      almostEqual(Array[i].x, object.x) &&
      almostEqual(Array[i].y, object.y)
    ) {
      return true;
    }
  }
  return false;
}

function almostEqual(a, b) {
  if (Math.abs(a - b) < 0.01) {
    return true;
  }
  return false;
}

function SAT_recieveNormals(A_translatedVertices, B_translatedVertices) {
  let normals = [new Vec2(1, 0), new Vec2(0, 1)];

  for (let i = 0; i < A_translatedVertices.length; i++) {
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

  for (let i = 0; i < B_translatedVertices.length; i++) {
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

function drawCircle(position, radius, color) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(position.x, position.y, radius, 0, 2 * Math.PI);
  ctx.fill();
}

function IntervalDistance(minA, maxA, minB, maxB) {
  if (minA > minB) {
    return minB - maxA;
  } else {
    return minA - maxB;
  }
}

function SAT(_M) {
  let _A = _M.A;
  let _B = _M.B;

  let normal = _B.transform.position.subtract(_A.transform.position);

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

    a_min = DotProduct(GetSupport(A_translatedVertices, axis.mult(1)), axis);

    a_max = DotProduct(GetSupport(A_translatedVertices, axis.mult(-1)), axis);

    b_min = DotProduct(GetSupport(B_translatedVertices, axis.mult(1)), axis);

    b_max = DotProduct(GetSupport(B_translatedVertices, axis.mult(-1)), axis);

    overlap = IntervalDistance(a_min, a_max, b_min, b_max);

    if (overlap < 0) {
      return false;
    }
    overlapList.push(overlap);
  }

  _M.penetrationDepth = Math.min(...overlapList);

  axis = axisList[overlapList.indexOf(M.penetrationDepth)];

  if (DotProduct(normal, axis) < 0) {
    _M.normal = axis.mult(-1);
  } else {
    _M.normal = axis;
  }

  return true;
}

function CirclevsOBB(_M) {
  let _A = _M.A;
  let _B = _M.B;

  vertices = translateMat(
    MAT22xMAT22(_B.shape.vertices, rotationMat(_B.transform.rotation)),
    _B.transform.position
  );

  let closestDist = Infinity;
  let closestPoint;

  normals = [];
  faces = [];
  collisionNormal = new Vec2(
    _A.transform.position.x - _B.transform.position.x,
    _A.transform.position.y - _B.transform.position.y
  );

  for (let i = 0; i < vertices.length; i++) {
    vertex_A = new Vec2(vertices[i][0], vertices[i][1]);
    vertex_B = new Vec2(
      vertices[(i + 1) % vertices.length][0],
      vertices[(i + 1) % vertices.length][1]
    );

    face_normal = new Vec2(vertex_B.x - vertex_A.x, vertex_B.y - vertex_A.y)
      .normalize()
      .normal()
      .mult(-1);

    if (DotProduct(collisionNormal, face_normal) > 0) {
      // drawLine(vertex_A, face_normal, 20, "red");

      faces.push([vertex_A, vertex_B, face_normal]);
    }
  }

  for (let i = 0; i < faces.length; i++) {
    distSqrd = pDistance(
      _A.transform.position.x,
      _A.transform.position.y,
      faces[i][0].x,
      faces[i][0].y,
      faces[i][1].x,
      faces[i][1].y
    );
    if (distSqrd[0] < closestDist) {
      closestDist = distSqrd[0];
      closestPoint = distSqrd[1];
    }

    if (closestDist < _A.shape.radius ** 2) {
      normal = new Vec2(
        closestPoint.x - _A.transform.position.x,
        closestPoint.y - _A.transform.position.y
      ).normalize();

      penetrationDepth = _A.shape.radius - Math.sqrt(closestDist);

      _M.penetrationDepth = penetrationDepth;
      _M.normal = normal;
      _M.contactCount = 1;
      _M.contactPoints = [closestPoint];

      // drawLine(closestPoint, _M.normal, 10, "red");

      return true;
    }
  }
  return false;
}

function findContactPoints(_M) {
  // Only runs if collision

  _A = _M.A;
  _B = _M.B;

  //CIRCLE

  if (_A.shape.type == "Circle" && _B.shape.type == "Circle") {
    ctx.strokeStyle = "red";

    contact1 = new Vec2(
      _M.normal.x * _A.shape.radius + _A.transform.position.x,
      _M.normal.y * _A.shape.radius + _A.transform.position.y
    );

    _M.contactCount = 1;
    _M.contactPoints = [contact1];

    return;
  }

  //CIRCLE and OBB

  if (_A.shape.type == "OBB" && _B.shape.type == "OBB") {
    //OBB

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
        edgePointB =
          B_translatedVertices[(j + 1) % B_translatedVertices.length];

        distance = pDistance(
          point[0],
          point[1],
          edgePointA[0],
          edgePointA[1],
          edgePointB[0],
          edgePointB[1]
        )[0];

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
        edgePointB =
          A_translatedVertices[(j + 1) % A_translatedVertices.length];

        distance = pDistance(
          point[0],
          point[1],
          edgePointA[0],
          edgePointA[1],
          edgePointB[0],
          edgePointB[1]
        )[0];

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
      contact1 = new Vec2(contact1[0], contact1[1]);

      // drawLine(contact1, _M.normal, 10, "red");

      _M.contactPoints = [contact1];
      _M.contactCount = 1;
      return;
    } else if (pointCount == 2) {
      contact1 = new Vec2(contact1[0], contact1[1]);
      contact2 = new Vec2(contact2[0], contact2[1]);

      // drawLine(contact1, _M.normal, 10, "red");
      contact2, _M.normal, 10, "red";

      _M.contactPoints = [contact1, contact2];
      _M.contactCount = 2;

      return;
    }
  }
}

function DotProduct(a, b) {
  return a.x * b.x + a.y * b.y;
}

function collisionResolve(_M) {
  let _A = _M.A;
  let _B = _M.B;

  if (!_M.contactPoints) {
    findContactPoints(_M);
  }
  PositionalCorrection(_M);

  let normal = _M.normal;

  // console.log(_M);

  for (let i = 0; i < _M.contactCount; i++) {
    const contactPoint = _M.contactPoints[i];
    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(contactPoint.x, contactPoint.y, 2, 0, Math.PI * 2);
    // ctx.fill();
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

  percent = 1; // usually 20% to 80%
  buffer = 0.01;

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

let drawList = [];

let i = 0;

class Manifold {
  A;
  B;
  penetrationDepth;
  normal;
  contactCount;
  contactPoints;

  constructor(_A, _B) {
    if (_A.shape.type == "Circle") {
      this.A = _A;
      this.B = _B;
    } else {
      this.A = _B;
      this.B = _A;
    }
  }

  update() {
    if (this.A.mass == 0 && this.B.mass == 0) {
      return;
    }
    if (this.A.type == this.B.type) {
      if (this.A.type !== "box") {
        return;
      }
    }

    // if (this.A.shape.type == "Circle" && this.B.shape.type == "Circle") {
    if (this.A.shape.type == "OBB" && this.B.shape.type == "OBB") {
      if (SAT(this)) {
        collisionResolve(this);
      }
    }
    // }

    if (this.A.shape.type == "Circle" && this.B.shape.type == "Circle") {
      if (CirclevsCircle(this)) {
        collisionResolve(this);
      }
    }

    if (this.A.shape.type == "Circle" && this.B.shape.type == "OBB") {
      if (CirclevsOBB(this)) {
        collisionResolve(this);
      }
    }
  }
}

new rigidbody(
  new OBB(
    [
      [0, 1],
      [30, 1],
      [30, 0],
      [0, 0],
    ],
    30
  ),
  500,
  500,
  "Static",
  0
);

mousePos = new Vec2(0, 0);

// B = new rigidbody(
//   new OBB(
//     [
//       [0, 1],
//       [1, 1],
//       [1, 0],
//       [0, 0],
//     ],
//     30
//   ),
//   550,
//   300,
//   "Static",
//   "box"
// );

document.addEventListener("mousemove", function (e) {
  mousePos.x = e.offsetX;
  mousePos.y = e.offsetY;
  // B.transform.position = mousePos;
});

document.addEventListener("mousedown", function (e) {
  // B.linearVelocity.x += 10;
  if (currentType == "Circle") {
    new rigidbody(
      new Circle(Math.random() * 25 + 10),
      mousePos.x,
      mousePos.y,
      "Rock",
      "box"
    );
    objects[objects.length - 1].transform.rotation =
      Math.random() * 2 * Math.PI;
  }
  if (currentType == "OBB1") {
    new rigidbody(
      new OBB(
        [
          [0, 2],
          [5, 2],
          [6, 0],
          [4, -1],
          [2, -1],
          [0, 0],
        ],
        20
      ),
      mousePos.x,
      mousePos.y,
      "Rock",
      "box"
    );
  }

  if (currentType == "OBB2") {
    for (let i = 0; i < 10; i++) {
      new rigidbody(
        new OBB(
          [
            [0, 1],
            [1, 1],
            [1, 0],
            [0, 0],
          ],
          20
        ),
        mousePos.x + Math.random() * 50 - 25,
        mousePos.y + Math.random() * 50 - 25,
        "Rock",
        "box"
      );
      objects[objects.length - 1].transform.rotation = Math.random() * 50;
    }
  }
});

class car {
  rb;

  constructor(x, y) {
    this.rb = new rigidbody(
      new OBB(
        [
          [0, 1],
          [3, 1],
          [3, 0],
          [0, 0],
        ],
        30
      ),
      x,
      y,
      new material_data("metal"),
      "car"
    );
  }
}

document.addEventListener("keydown", function (e) {
  if (e.key == "h") {
    currentType = "Circle";
  }
  if (e.key == "j") {
    currentType = "OBB1";
  }
  if (e.key == "k") {
    currentType = "OBB2";
  }
  if (e.key == "d") {
    Car.rb.linearVelocity.x += 0.1;
  }
  if (e.key == "a") {
    Car.rb.linearVelocity.x -= 0.1;
  }

  if (e.key == "x") {
    breakObject(objects[objects.length - 1]);
  }
});

function clear() {
  ctx.fillStyle = "#262626";
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
  // dt = 1;
  oldTime = Date.now() / 1000;
  clear();
  drawList.forEach((Object) => {
    ctx.fillStyle = "blue ";
    ctx.beginPath();
    ctx.arc(Object.x, Object.y, 3, 0, 2 * Math.PI);
    ctx.fill();
  });
  objects.forEach((Object) => {
    Object.update();
    Object.draw();

    Object.force.y += 100 * Object.mass_data.mass;

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
