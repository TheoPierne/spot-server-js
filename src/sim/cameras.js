'use strict';

const { Buffer } = require('node:buffer');

const sharp = require('sharp');

const { secToDuration, secToTimestamp } = require('./clock');
const {
  frameTreeSnapshot,
  pose,
  poseInv,
  poseMul,
  quatFromAxisAngle,
  quatFromEulerZXY,
  quatMul,
  quatRotate,
} = require('./math');
const geometryPb = require('../bosdyn/api/geometry_pb');
const imagePb = require('../bosdyn/api/image_pb');

const { Image, ImageSource, ImageResponse } = imagePb;
const { Format, PixelFormat } = Image;

// Range of the depth cameras (m).
const DEPTH_MAX_RANGE = 10;

/**
 * The pose of a camera in the body frame: position, and the direction it looks (yaw, pitch down, roll about the
 * optical axis). The optical frame has z forward, x right and y down.
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @param {number} yaw
 * @param {number} pitchDown
 * @param {number} [roll=0]
 * @returns {import('./math').Pose}
 */
function cameraMount(x, y, z, yaw, pitchDown, roll = 0) {
  // Optical frame from a frame looking along x: z_optical = x, x_optical = -y, y_optical = -z.
  const lookX = { w: 0.5, x: -0.5, y: 0.5, z: -0.5 };
  const direction = quatFromEulerZXY(yaw, 0, pitchDown);
  return pose(x, y, z, quatMul(quatMul(direction, lookX), quatFromAxisAngle({ x: 0, y: 0, z: 1 }, roll)));
}

/**
 * @param {string} name
 * @param {number} cols
 * @param {number} rows
 * @param {number} focal
 * @param {object} options
 * @returns {object} A camera source.
 */
function source(name, cols, rows, focal, options) {
  return { name, cols, rows, fx: focal, fy: focal, cx: cols / 2, cy: rows / 2, ...options };
}

/**
 * The image sources of a Spot: the five stereo cameras of the body (grayscale fisheye, depth, depth in the visual
 * frame), and the cameras of the gripper.
 * @param {boolean} hasArm
 * @returns {object[]}
 */
function imageSources(hasArm) {
  const body = [
    { name: 'frontleft', mount: cameraMount(0.415, 0.037, -0.023, -0.35, 0.35, 1.36) },
    { name: 'frontright', mount: cameraMount(0.415, -0.037, -0.023, 0.35, 0.35, 1.78) },
    { name: 'left', mount: cameraMount(-0.165, 0.11, 0.035, Math.PI / 2, 0.3) },
    { name: 'right', mount: cameraMount(-0.165, -0.11, 0.035, -Math.PI / 2, 0.3, Math.PI) },
    { name: 'back', mount: cameraMount(-0.425, 0, 0.035, Math.PI, 0.3) },
  ];
  const sources = [];
  for (const camera of body) {
    const visual = { mount: camera.mount, frame: `${camera.name}_fisheye`, parent: 'body' };
    sources.push(
      source(`${camera.name}_fisheye_image`, 640, 480, 330, {
        ...visual,
        type: 'visual',
        pixelFormats: [PixelFormat.PIXEL_FORMAT_GREYSCALE_U8],
      }),
      source(`${camera.name}_depth`, 424, 240, 212, {
        mount: camera.mount,
        frame: camera.name,
        parent: 'body',
        type: 'depth',
      }),
      source(`${camera.name}_depth_in_visual_frame`, 640, 480, 330, { ...visual, type: 'depth' }),
    );
  }
  if (hasArm) {
    // The cameras of the gripper look along the x axis of the hand.
    const color = {
      mount: pose(0.02, 0, 0.04, { w: 0.5, x: -0.5, y: 0.5, z: -0.5 }),
      frame: 'hand_color_image_sensor',
      parent: 'hand',
    };
    const depth = {
      mount: pose(0.02, 0.03, 0.04, { w: 0.5, x: -0.5, y: 0.5, z: -0.5 }),
      frame: 'hand_depth_sensor',
      parent: 'hand',
    };
    sources.push(
      source('hand_color_image', 640, 480, 552, {
        ...color,
        type: 'visual',
        color: true,
        pixelFormats: [PixelFormat.PIXEL_FORMAT_RGB_U8, PixelFormat.PIXEL_FORMAT_GREYSCALE_U8],
      }),
      source('hand_image', 224, 171, 208, {
        ...depth,
        type: 'visual',
        pixelFormats: [PixelFormat.PIXEL_FORMAT_GREYSCALE_U8],
      }),
      source('hand_depth', 224, 171, 208, { ...depth, type: 'depth' }),
      source('hand_depth_in_hand_color_frame', 640, 480, 552, { ...color, type: 'depth' }),
      source('hand_color_in_hand_depth_frame', 224, 171, 208, {
        ...depth,
        type: 'visual',
        color: true,
        pixelFormats: [PixelFormat.PIXEL_FORMAT_RGB_U8, PixelFormat.PIXEL_FORMAT_GREYSCALE_U8],
      }),
    );
  }
  for (const src of sources) {
    if (src.type === 'depth') src.pixelFormats = [PixelFormat.PIXEL_FORMAT_DEPTH_U16];
    src.imageFormats = src.type === 'depth' ? [Format.FORMAT_RAW] : [Format.FORMAT_JPEG, Format.FORMAT_RAW];
  }
  return sources;
}

/**
 * A pattern of 8x8 cells that looks like an AprilTag (black border, bits from the id).
 * @param {number} id
 * @returns {boolean[]} True for the white cells.
 */
function tagPattern(id) {
  const cells = [];
  let seed = (id * 2654435761) >>> 0;
  for (let row = 0; row < 8; row++) {
    for (let col = 0; col < 8; col++) {
      if (row === 0 || col === 0 || row === 7 || col === 7) {
        cells.push(false);
      } else {
        seed = (seed * 1103515245 + 12345) >>> 0;
        cells.push(((seed >>> 16) & 1) === 1);
      }
    }
  }
  return cells;
}

/**
 * The cameras of the robot: they render the room of the simulation (checkered floor, walls, fiducials, dock) by ray
 * casting from the pose of the robot, so the images change when the robot moves.
 */
class Cameras {
  /**
   * @param {import('../robot')} robot
   */
  constructor(robot) {
    this.robot = robot;
    this.sources = imageSources(Boolean(robot.arm));
    this.patterns = new Map();
  }

  /**
   * @param {string} name
   * @returns {?object}
   */
  source(name) {
    return this.sources.find(src => src.name === name) ?? null;
  }

  /**
   * @param {object} src
   * @returns {ImageSource}
   */
  sourceToProto(src) {
    const intrinsics = new ImageSource.PinholeModel.CameraIntrinsics()
      .setFocalLength(new geometryPb.Vec2().setX(src.fx).setY(src.fy))
      .setPrincipalPoint(new geometryPb.Vec2().setX(src.cx).setY(src.cy))
      .setSkew(new geometryPb.Vec2().setX(0).setY(0));
    const proto = new ImageSource()
      .setName(src.name)
      .setCols(src.cols)
      .setRows(src.rows)
      .setPinhole(new ImageSource.PinholeModel().setIntrinsics(intrinsics))
      .setImageType(
        src.type === 'depth' ? ImageSource.ImageType.IMAGE_TYPE_DEPTH : ImageSource.ImageType.IMAGE_TYPE_VISUAL,
      )
      .setPixelFormatsList(src.pixelFormats)
      .setImageFormatsList(src.imageFormats);
    if (src.type === 'depth') proto.setDepthScale(1000);
    return proto;
  }

  /**
   * The pose of a camera in the world.
   * @param {object} src
   * @returns {import('./math').Pose}
   */
  cameraInWorld(src) {
    const parent = this.robot.body.frameInWorld(src.parent);
    return poseMul(parent, src.mount);
  }

  /**
   * Renders a camera: intensities (and colors) and depths (along the optical axis).
   * @param {object} src
   * @returns {{gray: Uint8Array, rgb: ?Uint8Array, depth: Float32Array}}
   */
  render(src) {
    const { cols, rows } = src;
    const camera = this.cameraInWorld(src);
    const room = this.robot.config.world.room;
    const fiducials = this.robot.world.fiducials.map(fiducial => {
      const x = quatRotate(fiducial.pose.rot, { x: 1, y: 0, z: 0 });
      const y = quatRotate(fiducial.pose.rot, { x: 0, y: 1, z: 0 });
      const n = quatRotate(fiducial.pose.rot, { x: 0, y: 0, z: 1 });
      return {
        cx: fiducial.pose.x,
        cy: fiducial.pose.y,
        cz: fiducial.pose.z,
        ax: x.x,
        ay: x.y,
        az: x.z,
        bx: y.x,
        by: y.y,
        bz: y.z,
        nx: n.x,
        ny: n.y,
        nz: n.z,
        half: (this.robot.config.world.fiducialSizeMm / 1000) * 0.62,
        pattern: this._pattern(fiducial.id),
      };
    });
    const towers = this.robot.world.docks.map(dock => ({
      cx: dock.base.x - 0.5 * Math.cos(dock.base.yaw),
      cy: dock.base.y - 0.5 * Math.sin(dock.base.yaw),
      c: Math.cos(dock.base.yaw),
      s: Math.sin(dock.base.yaw),
    }));
    const gray = new Uint8Array(cols * rows);
    const rgb = src.color ? new Uint8Array(cols * rows * 3) : null;
    const depth = new Float32Array(cols * rows);
    // Columns of the rotation of the camera: the ray of a pixel is R * ((u - cx) / fx, (v - cy) / fy, 1), so the
    // parameter of the intersections is the depth along the optical axis.
    const r0 = quatRotate(camera.rot, { x: 1, y: 0, z: 0 });
    const r1 = quatRotate(camera.rot, { x: 0, y: 1, z: 0 });
    const r2 = quatRotate(camera.rot, { x: 0, y: 0, z: 1 });
    const ox = camera.x;
    const oy = camera.y;
    const oz = camera.z;
    for (let v = 0; v < rows; v++) {
      const ly = (v + 0.5 - src.cy) / src.fy;
      for (let u = 0; u < cols; u++) {
        const lx = (u + 0.5 - src.cx) / src.fx;
        const dx = r0.x * lx + r1.x * ly + r2.x;
        const dy = r0.y * lx + r1.y * ly + r2.y;
        const dz = r0.z * lx + r1.z * ly + r2.z;
        // The walls, the floor and the ceiling of the room: the first plane through which the ray leaves the box.
        let t = Infinity;
        let surface = 0;
        if (dz < -1e-9) {
          t = -oz / dz;
          surface = 1;
        } else if (dz > 1e-9) {
          t = (room.height - oz) / dz;
          surface = 2;
        }
        let tw = Infinity;
        if (dx > 1e-9) tw = (room.maxX - ox) / dx;
        else if (dx < -1e-9) tw = (room.minX - ox) / dx;
        if (tw < t) {
          t = tw;
          surface = 3;
        }
        tw = Infinity;
        if (dy > 1e-9) tw = (room.maxY - oy) / dy;
        else if (dy < -1e-9) tw = (room.minY - oy) / dy;
        if (tw < t) {
          t = tw;
          surface = 3;
        }
        let shade = 0;
        // The fiducials (only their front face).
        for (let i = 0; i < fiducials.length; i++) {
          const f = fiducials[i];
          const denominator = dx * f.nx + dy * f.ny + dz * f.nz;
          if (denominator > -1e-6) continue;
          const tf = ((f.cx - ox) * f.nx + (f.cy - oy) * f.ny + (f.cz - oz) * f.nz) / denominator;
          if (tf <= 0 || tf >= t) continue;
          const px = ox + dx * tf - f.cx;
          const py = oy + dy * tf - f.cy;
          const pz = oz + dz * tf - f.cz;
          const a = px * f.ax + py * f.ay + pz * f.az;
          const b = px * f.bx + py * f.by + pz * f.bz;
          if (a > f.half || a < -f.half || b > f.half || b < -f.half) continue;
          t = tf;
          surface = 5;
          // A white margin around the 8x8 cells.
          const row = Math.floor(((f.half - a) / (2 * f.half) - 0.1) / 0.1);
          const col = Math.floor(((f.half - b) / (2 * f.half) - 0.1) / 0.1);
          const inside = row >= 0 && row < 8 && col >= 0 && col < 8;
          shade = !inside || f.pattern[row * 8 + col] ? 235 : 20;
        }
        // The towers of the docks: boxes 0.19 m deep, 0.6 m wide and 0.6 m high, behind the fiducials.
        for (let i = 0; i < towers.length; i++) {
          const tower = towers[i];
          const lox = tower.c * (ox - tower.cx) + tower.s * (oy - tower.cy);
          const loy = -tower.s * (ox - tower.cx) + tower.c * (oy - tower.cy);
          const ldx = tower.c * dx + tower.s * dy;
          const ldy = -tower.s * dx + tower.c * dy;
          let near = 0;
          let far = t;
          let hit = true;
          const slab = (origin, direction, min, max) => {
            if (Math.abs(direction) < 1e-9) {
              if (origin < min || origin > max) hit = false;
              return;
            }
            let t1 = (min - origin) / direction;
            let t2 = (max - origin) / direction;
            if (t1 > t2) [t1, t2] = [t2, t1];
            if (t1 > near) near = t1;
            if (t2 < far) far = t2;
          };
          slab(lox, ldx, -0.1, 0.09);
          slab(loy, ldy, -0.3, 0.3);
          slab(oz, dz, 0, 0.6);
          if (hit && near > 0 && near <= far && near < t) {
            t = near;
            surface = 6;
            shade = 55;
          }
        }
        const hx = ox + dx * t;
        const hy = oy + dy * t;
        const hz = oz + dz * t;
        let red = -1;
        let green = 0;
        let blue = 0;
        if (surface === 1) {
          // Checkered floor (1 m tiles), with lines.
          const line = hx - Math.floor(hx) < 0.02 || hy - Math.floor(hy) < 0.02;
          const tile = (Math.floor(hx) + Math.floor(hy)) & 1;
          shade = line ? 60 : tile ? 150 : 115;
          if (line) [red, green, blue] = [55, 55, 60];
          else if (tile) [red, green, blue] = [170, 150, 120];
          else [red, green, blue] = [130, 112, 90];
        } else if (surface === 2) {
          shade = 205;
        } else if (surface === 3) {
          shade = hz < 0.1 ? 90 : 180 - Math.min(40, hz * 12);
          if (hz >= 0.1) [red, green, blue] = [200 - hz * 10, 205 - hz * 10, 215 - hz * 10];
        }
        // Darker far away.
        const falloff = Math.max(0.55, 1 - t / 25);
        const index = v * cols + u;
        gray[index] = Math.max(0, Math.min(255, Math.round(shade * falloff)));
        if (rgb) {
          if (red < 0) [red, green, blue] = [shade, shade, shade];
          rgb[index * 3] = Math.round(red * falloff);
          rgb[index * 3 + 1] = Math.round(green * falloff);
          rgb[index * 3 + 2] = Math.round(blue * falloff);
        }
        depth[index] = t;
      }
    }
    return { gray, rgb, depth };
  }

  _pattern(id) {
    if (!this.patterns.has(id)) this.patterns.set(id, tagPattern(id));
    return this.patterns.get(id);
  }

  /**
   * The frame tree of an image: the body, the inertial frames, the sensor.
   * @param {object} src
   * @returns {geometryPb.FrameTreeSnapshot}
   */
  _snapshot(src) {
    const edges = this.robot.body.frameTreeEdges();
    const bodyWorld = this.robot.body.bodyPoseWorld();
    edges[src.frame] = ['body', poseMul(poseInv(bodyWorld), this.cameraInWorld(src))];
    return frameTreeSnapshot(edges);
  }

  /**
   * GetImage, for one request.
   * @param {imagePb.ImageRequest} request
   * @returns {Promise<ImageResponse>}
   */
  async capture(request) {
    const { Status } = ImageResponse;
    const response = new ImageResponse();
    const src = this.source(request.getImageSourceName());
    if (!src) return response.setStatus(Status.STATUS_UNKNOWN_CAMERA);
    response.setSource(this.sourceToProto(src));

    // The format: the requested one or a fallback, by default JPEG for the visual images and RAW for the depth.
    const requested = request.getImageFormat();
    let format;
    if (requested === Format.FORMAT_UNKNOWN) {
      format = src.imageFormats[0];
    } else {
      format = [requested, ...request.getFallbackFormatsList()].find(f => src.imageFormats.includes(f));
      if (format === undefined) return response.setStatus(Status.STATUS_UNSUPPORTED_IMAGE_FORMAT_REQUESTED);
    }
    let pixelFormat = request.getPixelFormat();
    if (pixelFormat === PixelFormat.PIXEL_FORMAT_UNKNOWN) pixelFormat = src.pixelFormats[0];
    if (!src.pixelFormats.includes(pixelFormat)) {
      return response.setStatus(Status.STATUS_UNSUPPORTED_PIXEL_FORMAT_REQUESTED);
    }
    let ratio = request.getResizeRatio();
    if (ratio === 0) ratio = 1;
    if (!(ratio > 0 && ratio <= 1)) return response.setStatus(Status.STATUS_UNSUPPORTED_RESIZE_RATIO_REQUESTED);
    const quality = request.getQualityPercent() > 0 ? Math.min(100, request.getQualityPercent()) : 75;

    const now = this.robot.clock.now();
    const rendered = this.render(src);
    const width = Math.max(1, Math.round(src.cols * ratio));
    const height = Math.max(1, Math.round(src.rows * ratio));
    let data;
    if (pixelFormat === PixelFormat.PIXEL_FORMAT_DEPTH_U16) {
      const raw = Buffer.alloc(src.cols * src.rows * 2);
      for (let i = 0; i < rendered.depth.length; i++) {
        const mm =
          rendered.depth[i] > 0 && rendered.depth[i] < DEPTH_MAX_RANGE ? Math.round(rendered.depth[i] * 1000) : 0;
        raw.writeUInt16LE(mm, i * 2);
      }
      data = ratio === 1 ? raw : this._resizeDepth(raw, src.cols, src.rows, width, height);
    } else {
      const color = pixelFormat === PixelFormat.PIXEL_FORMAT_RGB_U8;
      const channels = color ? 3 : 1;
      let image = sharp(Buffer.from(color ? rendered.rgb : rendered.gray), {
        raw: { width: src.cols, height: src.rows, channels },
      });
      if (ratio !== 1) image = image.resize(width, height);
      data = format === Format.FORMAT_JPEG ? await image.jpeg({ quality }).toBuffer() : await image.raw().toBuffer();
    }
    const capture = new imagePb.ImageCapture()
      .setAcquisitionTime(secToTimestamp(now - 0.03))
      .setTransformsSnapshot(this._snapshot(src))
      .setFrameNameImageSensor(src.frame)
      .setImage(
        new Image()
          .setCols(width)
          .setRows(height)
          .setData(new Uint8Array(data))
          .setFormat(format)
          .setPixelFormat(pixelFormat),
      )
      .setCaptureParams(new imagePb.CaptureParameters().setExposureDuration(secToDuration(0.008)).setGain(1.5));
    response.setShot(capture).setStatus(Status.STATUS_OK);
    // The resized images report their size in the source too.
    if (ratio !== 1) response.getSource().setCols(width).setRows(height);
    return response;
  }

  _resizeDepth(raw, cols, rows, width, height) {
    const out = Buffer.alloc(width * height * 2);
    for (let v = 0; v < height; v++) {
      for (let u = 0; u < width; u++) {
        const su = Math.min(cols - 1, Math.floor((u * cols) / width));
        const sv = Math.min(rows - 1, Math.floor((v * rows) / height));
        out.writeUInt16LE(raw.readUInt16LE((sv * cols + su) * 2), (v * width + u) * 2);
      }
    }
    return out;
  }
}

module.exports = { Cameras, imageSources, tagPattern };
