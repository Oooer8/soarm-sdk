import * as THREE from "/vendor/three.module.js";
import URDFLoader from "/vendor/URDFLoader.js?v=soarm101-stl";

export async function createUrdfViewer(container, { urdfUrl, joints = {}, renderHz = 60 }) {
  const scene = new THREE.Scene();
  scene.background = null;

  const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
  camera.up.set(0, 1, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;

  container.replaceChildren(renderer.domElement);
  renderer.domElement.className = "urdf-canvas";

  const keyLight = new THREE.DirectionalLight(0xffffff, 2.6);
  keyLight.position.set(1.5, 2.0, 1.6);
  scene.add(keyLight);
  scene.add(new THREE.HemisphereLight(0xdde8ff, 0x1d252f, 1.4));

  const grid = new THREE.GridHelper(0.7, 14, 0x496273, 0x2c3a45);
  grid.position.y = -0.002;
  scene.add(grid);

  const root = new THREE.Group();
  scene.add(root);

  const loader = new URDFLoader();
  const robot = await loader.loadAsync(urdfUrl);
  robot.rotation.x = -Math.PI / 2;
  root.add(robot);
  const controls = createCameraControls(camera, renderer.domElement);

  const applyJoints = (values) => {
    for (const [name, value] of Object.entries(values || {})) {
      if (robot.joints?.[name]) robot.setJointValue(name, Number(value));
    }
  };

  applyJoints(joints);
  controls.frame(frameRobot(robot));

  const resize = () => {
    const rect = container.getBoundingClientRect();
    const width = Math.max(1, rect.width);
    const height = Math.max(1, rect.height);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    controls.update();
  };
  const observer = new ResizeObserver(resize);
  observer.observe(container);
  resize();

  let disposed = false;
  let minFrameMs = 1000 / Math.max(1, Number(renderHz) || 60);
  let lastRenderAt = 0;
  const animate = (timestamp = 0) => {
    if (disposed) return;
    if (!lastRenderAt || timestamp - lastRenderAt >= minFrameMs - 0.5) {
      renderer.render(scene, camera);
      lastRenderAt = timestamp;
    }
    requestAnimationFrame(animate);
  };
  animate();

  return {
    setJoints: applyJoints,
    setRenderHz(value) {
      minFrameMs = 1000 / Math.max(1, Number(value) || 60);
    },
    dispose() {
      disposed = true;
      observer.disconnect();
      controls.dispose();
      renderer.dispose();
      container.replaceChildren();
    },
  };
}

function frameRobot(object) {
  const box = new THREE.Box3().setFromObject(object);
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  object.position.sub(center);

  return Math.max(size.x, size.y, size.z, 0.1);
}

function createCameraControls(camera, domElement) {
  const target = new THREE.Vector3(0, 0, 0);
  const right = new THREE.Vector3();
  const up = new THREE.Vector3();
  let yaw = -0.72;
  let pitch = 0.42;
  let distance = 1.0;
  let minDistance = 0.1;
  let maxDistance = 5.0;
  let mode = null;
  let lastX = 0;
  let lastY = 0;

  const update = () => {
    const cosPitch = Math.cos(pitch);
    camera.position.set(
      target.x + distance * cosPitch * Math.sin(yaw),
      target.y + distance * Math.sin(pitch),
      target.z + distance * cosPitch * Math.cos(yaw),
    );
    camera.lookAt(target);
    camera.updateMatrixWorld();
  };

  const frame = (radius) => {
    const safeRadius = Math.max(radius, 0.1);
    distance = safeRadius * 4.8;
    minDistance = safeRadius * 0.85;
    maxDistance = safeRadius * 12.0;
    camera.near = Math.max(0.001, safeRadius / 100);
    camera.far = safeRadius * 30;
    camera.updateProjectionMatrix();
    target.set(0, 0, 0);
    update();
  };

  const panCamera = (dx, dy) => {
    const rect = domElement.getBoundingClientRect();
    const height = Math.max(1, rect.height);
    const width = Math.max(1, rect.width);
    const viewHeight = 2 * distance * Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5));
    const viewWidth = viewHeight * camera.aspect;

    right.setFromMatrixColumn(camera.matrix, 0);
    up.setFromMatrixColumn(camera.matrix, 1);
    target.addScaledVector(right, (-dx / width) * viewWidth);
    target.addScaledVector(up, (dy / height) * viewHeight);
    update();
  };

  const onPointerDown = (event) => {
    event.preventDefault();
    mode = event.button === 1 || event.button === 2 || event.shiftKey ? "pan" : "orbit";
    lastX = event.clientX;
    lastY = event.clientY;
    domElement.setPointerCapture(event.pointerId);
  };

  const onPointerMove = (event) => {
    if (!mode) return;
    event.preventDefault();
    const dx = event.clientX - lastX;
    const dy = event.clientY - lastY;
    lastX = event.clientX;
    lastY = event.clientY;

    if (mode === "pan") {
      panCamera(dx, dy);
      return;
    }

    yaw -= dx * 0.008;
    pitch = THREE.MathUtils.clamp(pitch + dy * 0.006, -1.25, 1.25);
    update();
  };

  const onPointerUp = (event) => {
    mode = null;
    if (domElement.hasPointerCapture(event.pointerId)) {
      domElement.releasePointerCapture(event.pointerId);
    }
  };

  const onWheel = (event) => {
    event.preventDefault();
    const scale = Math.exp(event.deltaY * 0.001);
    distance = THREE.MathUtils.clamp(distance * scale, minDistance, maxDistance);
    update();
  };

  const onContextMenu = (event) => event.preventDefault();

  domElement.addEventListener("pointerdown", onPointerDown);
  domElement.addEventListener("pointermove", onPointerMove);
  domElement.addEventListener("pointerup", onPointerUp);
  domElement.addEventListener("pointercancel", onPointerUp);
  domElement.addEventListener("wheel", onWheel, { passive: false });
  domElement.addEventListener("contextmenu", onContextMenu);

  update();

  return {
    frame,
    update,
    dispose() {
      domElement.removeEventListener("pointerdown", onPointerDown);
      domElement.removeEventListener("pointermove", onPointerMove);
      domElement.removeEventListener("pointerup", onPointerUp);
      domElement.removeEventListener("pointercancel", onPointerUp);
      domElement.removeEventListener("wheel", onWheel);
      domElement.removeEventListener("contextmenu", onContextMenu);
    },
  };
}
