// Enabling debug controls
const ENABLE_ORBIT_CONTROLS = false;
const ENABLE_KEYBOARD_DEBUGGING = false;
const DEBUG_ROBOT_MODEL_POSE = false;

import { NetworkTables } from '/lib/networktables.js';
import * as THREE from '/lib/three.js';
import { OrbitControls } from '/lib/OrbitControls.js';
import { STLLoader } from '/lib/STLLoader.js'
import { TextSprite } from '/lib/TextSprite.js';

// Initialize parameters
let lastShoulderPosition = 0;
let lastElbowPosition = 0;
let lastWristValue = 0;
let lastClawAlpha = 0;
let lastHarvesterAlpha = 0;
let debugRobotModelPoseCount = 0;

let scoring = null
let currentFrustumScale = 1000;
const currentSelection = {
    x: 0,
    y: 0
};

// Set up internal clock
const clock = new THREE.Clock();
const frameInterval = 1 / 30;
let frameDelta = 0;

const loader = new STLLoader();

//Set up renderer
const renderer = new THREE.WebGLRenderer({
    alpha: true,
    antialias: true,
    powerPreference: `high-performance`
});
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Set up camera position
const frustum = calculateCameraFrustum(currentFrustumScale);
const camera = new THREE.OrthographicCamera(frustum.left, frustum.right, frustum.top, frustum.bottom, 1, 100);
camera.position.z = 10;

const raycaster = new THREE.Raycaster();

const scene = new THREE.Scene();

// Load cone and cube STL files
const coneSTL = await loadSTL(`./assets/models/cone.stl`);
const cubeSTL = await loadSTL(`./assets/models/cube.stl`);

// Set up all options for where you are scoring
const selectionOptions = [];
for (let i = 0; i < 3; i++) {
    selectionOptions.push([]);
    for (let j = 0; j < 9; j++) {
        const mesh = j % 3 === 1 ? createMeshFromSTL(cubeSTL) : createMeshFromSTL(coneSTL);
        mesh.position.set((j * 0.75) - 3, (i * -0.5) + 1.55, 0);
        mesh.rotation.set(Math.PI / 6, -Math.PI / 6, 0);
        mesh.optionData = { x: j, y: i }
        scene.add(mesh);
        selectionOptions[i].push(mesh);
    }
}

// Set up rounded rectangles behind everything
const selectionOptionBackgrounds = [];
for (let i = 0; i < 3; i++) {
    const mesh = new THREE.Mesh(
        roundedRectangle((i * 2.25) - 3.3125, 0.125, 2.125, 1.75, 0.2),
            new THREE.MeshBasicMaterial({
            color: new THREE.Color(0x000000),
            transparent: true,
            opacity: 0.1,
            side: THREE.DoubleSide
        })
    );
    mesh.position.z = -2;
    scene.add(mesh);
    selectionOptionBackgrounds.push(mesh);
}

// Set up models of different parts of the robot
const robotModels = {
    driveBase: createMeshFromSTL(await loadSTL(`./assets/models/drivebase.stl`)),
    shoulder: createMeshFromSTL(await loadSTL(`./assets/models/shoulder.stl`)),
    elbow: createMeshFromSTL(await loadSTL(`./assets/models/elbow.stl`)),
    wrist: createMeshFromSTL(await loadSTL(`./assets/models/wrist.stl`)),
    clawLeft: createMeshFromSTL(await loadSTL(`./assets/models/clawleft.stl`)),
    clawRight: createMeshFromSTL(await loadSTL(`./assets/models/clawright.stl`)),
    recordPlayer: createMeshFromSTL(await loadSTL(`./assets/models/recordPlayer.stl`)),
    harvester: createMeshFromSTL(await loadSTL(`./assets/models/harvester.stl`))
};

const harvesterLateralHelper = new THREE.Object3D();

// Initialize position data for parts of robot
const robotPoseData = {
    shoulder: { value: 0, since: Date.now(), startingValue: 0 },
    elbow: { value: 0, since: Date.now(), startingValue: 0 },
    wrist: { value: 0, since: Date.now(), startingValue: 0 },
    claw: { state: false, since: Date.now(), startingAlpha: 0 },
    recordPlayer: 0,
    harvester: { state: false, since: Date.now(), startingAlpha: 0 }
};

// Set up robot model
robotModels.driveBase.add(robotModels.shoulder);
robotModels.shoulder.add(robotModels.elbow);
robotModels.elbow.add(robotModels.wrist);
robotModels.wrist.add(robotModels.clawLeft).add(robotModels.clawRight);
robotModels.driveBase.add(robotModels.recordPlayer);
robotModels.driveBase.add(harvesterLateralHelper);
harvesterLateralHelper.add(robotModels.harvester);

robotModels.driveBase.material.uniforms.thickness.value = 0.3;

robotModels.driveBase.position.set(0, -1.6, 0);
robotModels.driveBase.rotation.set(0.3,  0.6, Math.PI);
robotModels.driveBase.geometry.scale(1.2, 1.2, 1.2);

// Set shoulder angle to its fixed angle (in radians)
setShoulder(-0.13);

scene.add(robotModels.driveBase);

// Set up text fields
const voltageText = new TextSprite(`0.0V`, 0.45, `#ffffff`);
voltageText.fontSize = 100;
voltageText.fontFace = `'Library'`;
voltageText.position.set(-2.4375, -0.7, 0);
scene.add(voltageText);

const psiText = new TextSprite(`0.0 PSI`, 0.3, `#ffffff`);
psiText.fontSize = 100;
psiText.fontFace = `'Library'`;
psiText.position.set(-2.4375, -1.125, 0);
scene.add(psiText);

const timerText = new TextSprite(`0:00`, 0.525, `#ffffff`);
timerText.fontSize = 100;
timerText.fontFace = `'Library'`;
timerText.position.set(2.4375, -0.875, 0);
scene.add(timerText);

for (let i = 0; i < 2; i++) {
    const mesh = new THREE.Mesh(
        roundedRectangle(i === 0 ? -3.3125 : 1.5625, -1.75, 1.75, 1.75, 0.2),
        new THREE.MeshBasicMaterial({
            color: new THREE.Color(0x000000),
            transparent: true,
            opacity: 0.1,
            side: THREE.DoubleSide
        })
    );
    mesh.position.z = -2;
    scene.add(mesh);
}

// Create loop to refresh the text once a second
setInterval(() => refreshText(), 1000);

// Integer enum: 0 = Red, 1 = Blue, 2 = Invalid
NetworkTables.addKeyListener(`/dashboard/general/alliance`, (_, value) => {
    if (typeof value === `number`) {
        selectionOptionBackgrounds.forEach((background, i) => {
            if (i !== 1) {
                background.material.color = value === 0 ? new THREE.Color(0xff0000) : (value === 1 ? new THREE.Color(0x0000ff) : new THREE.Color(0x000000));
            }
        });
    }
}, true);

// Double: voltage
NetworkTables.addKeyListener(`/dashboard/general/voltage`, (_, value) => {
    if (typeof value === `number`) {
        voltageText.text = `${(Math.round(value * 10) / 10).toFixed(1)}V`;
    };
}, true);

// Double: PSI
NetworkTables.addKeyListener(`/dashboard/general/psi`, (_, value) => {
    if (typeof value === `number`) {
        psiText.text = `${value.toFixed(1)} PSI`;
    }
}, true);

// Integer: match time in seconds
NetworkTables.addKeyListener(`/dashboard/general/time`, (_, value) => {
    if (typeof value === `number` && value >= 0 && value <= 300) {
        timerText.text = new Date(value * 1000).toISOString().substring(15, 19);
    }
});

// Integer tuple: [grid, column, row]
NetworkTables.addKeyListener(`/dashboard/target/selection`, (_, value) => {
    if (value instanceof Array) {
        currentSelection.x = (value[0] * 3) + value[1];
        currentSelection.y = value[2];
    }
}, true);

// Boolean: true = scoring, false = finished scoring
NetworkTables.addKeyListener(`/dashboard/target/scoring`, (_, value) => {
    if (typeof value === `boolean`) {
        if (value) startScoring();
        else finishScoring();
    }
}, false);

let last = Date.now();
// Double: radians
NetworkTables.addKeyListener(`/dashboard/robotmodel/elbow`, (_, value) => {
    if (typeof value === `number`) {
        robotPoseData.elbow = { value, since: Date.now(), startingValue: lastElbowPosition };
    }
    last = Date.now();
}, true);

// Number: length in meters
NetworkTables.addKeyListener(`/dashboard/robotmodel/wrist`, (_, value) => {
    if (typeof value === `number`) {
        robotPoseData.wrist = { value, since: Date.now(), startingValue: lastWristValue };
    }
});

// Boolean: open
NetworkTables.addKeyListener(`/dashboard/robotmodel/claw`, (_, value) => {
    if (typeof value === `boolean`) {
        robotPoseData.claw = { state: value, since: Date.now(), startingAlpha: lastClawAlpha };
    }
});

// Double: speed
NetworkTables.addKeyListener(`/dashboard/robotmodel/recordplayer`, (_, value) => {
    if (typeof value === `number`) {
        robotPoseData.recordPlayer = value;
    }
});

// Boolean: deployed
NetworkTables.addKeyListener(`/dashboard/robotmodel/harvester`, (_, value) => {
    if (typeof value === `boolean`) {
        robotPoseData.harvester = { state: value, since: Date.now(), startingAlpha: lastHarvesterAlpha };
    }
});

// If window is resized rerender
window.addEventListener(`resize`, () => {
    updateCameraFrustum();
    renderer.setSize(window.innerWidth, window.innerHeight);
    render();
});

// Handle click events
document.addEventListener(`click`, (event) => {
    const clickX = (event.clientX / window.innerWidth) * 2 - 1;
    const clickY = - (event.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera({ x: clickX, y: clickY }, camera);
    const raycasterIntersects = raycaster.intersectObjects(scene.children, false)?.filter((mesh) => selectionOptions.flat().includes(mesh.object))[0]?.object?.optionData;

    if (raycasterIntersects) {
        currentSelection.x = raycasterIntersects.x;
        currentSelection.y = raycasterIntersects.y;
        NetworkTables.putValue(`/dashboard/target/touchselection`, [Math.floor(raycasterIntersects.x / 3), raycasterIntersects.x % 3, raycasterIntersects.y]);
    }
});

// If in keyboard debugging mode handle necessary key presses
if (ENABLE_KEYBOARD_DEBUGGING) {
    document.onkeydown = (event) => {
        if (!scoring) {
            if (event.key === `ArrowUp`) currentSelection.y--;
            if (event.key === `ArrowDown`) currentSelection.y++;
            if (event.key === `ArrowLeft`) currentSelection.x--;
            if (event.key === `ArrowRight`) currentSelection.x++;
            currentSelection.x = Math.min(Math.max(currentSelection.x, 0), 8);
            currentSelection.y = Math.min(Math.max(currentSelection.y, 0), 2);

            if (event.key === `Enter`) {
                startScoring();
                setTimeout(() => finishScoring(), 2000);
            }
        }
    };
}

// If orbit debugging controls are on handle rerendering at different camera angles
if (ENABLE_ORBIT_CONTROLS) {
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enablePan = false;
    controls.enableZoom = true;
}

animate();

/** Turn dimensions into shape */
function roundedRectangle (x, y, width, height, radius) {
    const shape = new THREE.Shape();
    shape.moveTo(x, y + radius);
    shape.lineTo(x, y + height - radius);
    shape.quadraticCurveTo(x, y + height, x + radius, y + height);
    shape.lineTo(x + width - radius, y + height);
    shape.quadraticCurveTo(x + width, y + height, x + width, y + height - radius);
    shape.lineTo(x + width, y + radius);
    shape.quadraticCurveTo(x + width, y, x + width - radius, y);
    shape.lineTo(x + radius, y);
    shape.quadraticCurveTo(x, y, x, y + radius);
    return new THREE.ShapeGeometry(shape)
}

/** Load STL asynchronously (using promises) */
async function loadSTL (path) {
    return await new Promise((resolve) => {
        loader.load(path, (geometry) => {
            resolve(geometry);
        });
    })
}

/** Use STL to create a mesh */
function createMeshFromSTL (geometry) {
    const geometryClone = geometry.clone();
    geometryClone.deleteAttribute(`normal`);
    geometryClone.deleteAttribute(`uv`);

    const position = geometryClone.attributes.position;
    const centers = new Float32Array(position.count * 3);
    const vectors = [
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 1)
    ];

    for (let i = 0, l = position.count; i < l; i++) {
        vectors[i % 3].toArray(centers, i * 3);
    }

    geometryClone.setAttribute(`center`, new THREE.BufferAttribute(centers, 3));
    geometryClone.scale(0.001, 0.001, 0.001)

    const material = new THREE.ShaderMaterial({
        uniforms: {
            thickness: { value: 1 },
            color: { type: `vec3`, value: new THREE.Color(0xffffff) }
        },
        vertexShader: document.getElementById(`vertexShader`).textContent,
        fragmentShader: document.getElementById(`fragmentShader`).textContent,
        side: THREE.DoubleSide,
        alphaToCoverage: true
    });
    material.extensions.derivatives = true;

    return new THREE.Mesh(geometryClone, material);
}

/** Animate the robot model based on frame rate */
function animate() {
    // TODO: ask Bryce what this does
    requestAnimationFrame(animate);

    // If hasn't been long enough to animate next frame return
    frameDelta += clock.getDelta();
    if (frameDelta <= frameInterval) return;
    frameDelta = frameDelta % frameInterval;

    // When program started zoom in
    if (currentFrustumScale !== 4) {
        currentFrustumScale += (4 - currentFrustumScale) * 0.3;
        updateCameraFrustum();
    }

    // If debugging robot model run animations on all parts of model otherwise use network table
    if (DEBUG_ROBOT_MODEL_POSE) {
        debugRobotModelPoseCount++;
        setElbow(-1.1 + Math.sin(debugRobotModelPoseCount / 50) * 1.4);
        setWrist(Math.min(Math.max(Math.sin(debugRobotModelPoseCount / 25), 0), 0.616));
        openClaw(Math.min(Math.max(Math.sin(debugRobotModelPoseCount / 25) * 5, 0), 1));
        spinRecordPlayer(0.04);
        deployHarvester(Math.min(Math.max(Math.sin(debugRobotModelPoseCount / 25) * 2, 0), 1));
    } else {
        setElbow(interpolateArm(robotPoseData.elbow, 500));
        setWrist(interpolateArm(robotPoseData.wrist, 100));
        openClaw(determineBooleanAlpha(robotPoseData.claw, 150));
        spinRecordPlayer((robotPoseData.recordPlayer / 30) * 2 * Math.PI);
        deployHarvester(determineBooleanAlpha(robotPoseData.harvester, 350));
    }

    // For each cube and cone if they are selected spin them and make them bigger
    selectionOptions.forEach((row, y) => {
        row.forEach((selectionOption, x) => {
            let rotationDelta;
            let targetScale;

            if (currentSelection.x === x && currentSelection.y === y) {
                if (scoring?.x === x && scoring?.y === y) {
                    rotationDelta = -0.6;
                    targetScale = 1.6;

                    const alpha = (Math.sin((Date.now() - scoring.since) / 75) + 1) / 2;
                    selectionOption.material.uniforms.color.value.lerpColors(new THREE.Color(0xffffff), new THREE.Color(x % 3 === 1 ? 0xa718b2 : 0xefdf0d), alpha);
                } else {
                    rotationDelta = -0.08;
                    targetScale = 1.3;
                }
            } else {
                targetScale = 1;
            }

            if (typeof rotationDelta === `number`) selectionOption.rotation.y += rotationDelta;
            else selectionOption.rotation.y = -Math.PI / 6;

            const newScale = selectionOption.scale.x + ((targetScale - selectionOption.scale.x) * 0.25)
            selectionOption.scale.set(newScale, newScale, newScale)
        });
    });

    render();
}

/** Render the graphics using camera location */
function render() {
    renderer.render(scene, camera);
}

/** Calculate field of view of camera */
function calculateCameraFrustum () {
    const aspect = window.innerWidth / window.innerHeight;
    return {
        left: currentFrustumScale * aspect / - 2,
        right: currentFrustumScale * aspect / 2,
        top: currentFrustumScale / 2,
        bottom: currentFrustumScale / - 2
    }
}

/** Update camera field of view */
function updateCameraFrustum () {
    const frustum = calculateCameraFrustum();
    camera.left = frustum.left;
    camera.right = frustum.right;
    camera.top = frustum.top;
    camera.bottom = frustum.bottom;
    camera.updateProjectionMatrix();
}

/** Set current selection as scoring */
function startScoring () {
    scoring = {
        x: currentSelection.x,
        y: currentSelection.y,
        since: Date.now()
    };
}

/** Set scored cone or cube as scored and reset scoring variable */
function finishScoring () {
    if (scoring && selectionOptions[scoring.y] && selectionOptions[scoring.y][scoring.x]) selectionOptions[scoring.y][scoring.x].material.uniforms.color.value = new THREE.Color(scoring.x % 3 === 1 ? 0xa718b2 : 0xefdf0d);
    scoring = null;
}

/** Rotate shoulder to input angle */
function setShoulder (theta) {
    function _moveShoulder (delta) {
        const origin = new THREE.Vector3(-0.13, -0.085, 0);
        const axis = new THREE.Vector3(0, 0, 1);

        robotModels.shoulder.position.sub(origin);
        robotModels.shoulder.position.applyAxisAngle(axis, delta);
        robotModels.shoulder.position.add(origin);

        robotModels.shoulder.rotateOnAxis(axis, delta);
    }

    _moveShoulder(-lastShoulderPosition);
    _moveShoulder(theta);
    lastShoulderPosition = theta;
}

/** Rotate elbow to input angle */
function setElbow (theta) {
    function _moveElbow (delta) {
        const origin = new THREE.Vector3(-0.13, -1.05, 0);
        const axis = new THREE.Vector3(0, 0, 1);
    
        robotModels.elbow.position.sub(origin);
        robotModels.elbow.position.applyAxisAngle(axis, -delta);
        robotModels.elbow.position.add(origin);
    
        robotModels.elbow.rotateOnAxis(axis, -delta);
    }

    _moveElbow(-lastElbowPosition + 0.13);
    _moveElbow(theta - 0.13);
    lastElbowPosition = theta;
}

/** Rotate elbow to input length */
function setWrist (length) {
    robotModels.wrist.position.set(0, length * 0.7, 0);
    lastWristValue = length;
}

/** Set claw as open or closed based on input and rotate both parts to match */
function openClaw (alpha) {
    const delta = (lastClawAlpha - alpha) * Math.PI / 4;

    [robotModels.clawLeft, robotModels.clawRight].forEach((claw, i) => {
        const origin = new THREE.Vector3(0.05, -.5, 0);
        const axis = new THREE.Vector3(1, 0, 0);

        claw.position.sub(origin);
        claw.position.applyAxisAngle(axis, delta * (i === 0 ? -1 : 1));
        claw.position.add(origin);

        claw.rotateOnAxis(axis, delta * (i === 0 ? -1 : 1));
        lastClawAlpha = alpha;
    });
}

/** Rotate record player at speed inputted */
function spinRecordPlayer (alpha) {
    const origin = new THREE.Vector3(0.089, 0, 0);
    const axis = new THREE.Vector3(0, 1, 0);

    robotModels.recordPlayer.position.sub(origin);
    robotModels.recordPlayer.position.applyAxisAngle(axis, alpha);
    robotModels.recordPlayer.position.add(origin);

    robotModels.recordPlayer.rotateOnAxis(axis, alpha);
}

/** Set harvester as in or out based on input and rotate to match */
function deployHarvester (alpha) {
    const delta = (lastHarvesterAlpha - alpha) * -Math.PI / 6;
    const origin = new THREE.Vector3(0.313, 0.3, 0);
    const axis = new THREE.Vector3(0, 0, 1);

    robotModels.harvester.position.sub(origin);
    robotModels.harvester.position.applyAxisAngle(axis, delta);
    robotModels.harvester.position.add(origin);

    robotModels.harvester.rotateOnAxis(axis, delta);
    harvesterLateralHelper.position.set(alpha * 0.035, 0, 0);
    
    lastHarvesterAlpha = alpha;
}

/** Interpolate value along movement based on inputted value and period */
function interpolateArm (poseData, period) {
    return poseData.value - (Math.max(1 - ((Date.now() - poseData.since) / period), 0) * (poseData.value - poseData.startingValue));
}

/** Interpolate value along movement based on which of two end states must be reached and period */
function determineBooleanAlpha (poseData, period) {
    const p = ((Date.now() - poseData.since) / period) + (poseData.state ? poseData.startingAlpha : 1 - poseData.startingAlpha);
    return Math.max(Math.min(poseData.state ? p : 1 - p, 1), 0);
}

/** Refresh all text being rendered */
function refreshText () {
    voltageText.text = voltageText.text;
    psiText.text = psiText.text;
    timerText.text = timerText.text;
}
