import './style.css';

import {
    AmbientLight,
    AxesHelper,
    BoxBufferGeometry,
    Color,
    DirectionalLight,
    DoubleSide,
    GridHelper,
    Mesh,
    MeshBasicMaterial,
    PerspectiveCamera,
    PlaneBufferGeometry,
    Raycaster,
    Scene,
    Vector2, Vector3,
    WebGLRenderer
} from 'three';
import { OrbitControls }    from 'three/examples/jsm/controls/OrbitControls';
import { OutlineEffect }    from 'three/examples/jsm/effects/OutlineEffect';
import { IKSolver, Solver }        from './solvers/IKSolver';
import { createExample } from './bones';

let renderer;
let effect;
let time = 0;
let canvas;
let mouseHasMoved = false;
let mouse = new Vector2();
let mouseRelative = new Vector2();
// let container;

let camera;
let scene;
let raycastPlane;
let mouseHelper;
let state = {
    animateBones: false,
    inverseBones: true
};
let skeleton;
let solver;
let targetPoint = new Vector3(0, 10, 0);

let cameras = [];
let scenes = [];
let raycastPlanes = [];
let mouseHelpers = [];
let states = [];
let mice = [];
let skeletons = [];
let solvers = [];
let targetPoints = [];

function init()
{
    // HTML
    canvas = document.getElementById('c');
    // container = document.createElement('div');
    // document.body.appendChild(container);

    // Renderer
    renderer = new WebGLRenderer({
        canvas: canvas,
        antialias: true,
    });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setClearColor(0x000000, 1);
    // renderer.setSize(window.innerWidth, window.innerHeight);
    // container.appendChild(renderer.domElement);
    effect = new OutlineEffect(renderer);

    // TODO iterate on scenes, store element

    // Scene, Camera, Controls, Lights
    let elementWidth = window.innerWidth;
    let elementHeight = window.innerHeight;

    scene = new Scene();
    scene.background = new Color(0x000000);
    camera = new PerspectiveCamera(45, elementWidth / elementHeight, 1, 2000);
    camera.position.z = 30;
    let controls = new OrbitControls(camera, renderer.domElement);
    controls.enablePan = false;
    // window.addEventListener('resize', onWindowResize, false);
    window.addEventListener('mousemove', onMouseMove);

    let ambient = new AmbientLight(0x666666);
    scene.add(ambient);
    let directionalLight = new DirectionalLight(0x887766);
    directionalLight.position.set(-1, 1, 1).normalize();
    scene.add(directionalLight);

    // Helpers
    let gridHelper = new GridHelper(30, 10);
    gridHelper.position.y = -10;
    scene.add(gridHelper);
    let axesHelper = new AxesHelper(5);
    axesHelper.position.y = -10;
    axesHelper.position.x = 10;
    axesHelper.position.z = 10;
    scene.add(axesHelper);

    let planeGeometry = new PlaneBufferGeometry(30, 30, 2);
    let planeMaterial = new MeshBasicMaterial({
        color: 0xffff00, side: DoubleSide, wireframe: true
    });
    raycastPlane = new Mesh(planeGeometry, planeMaterial);
    // raycastPlane.rotation.x = Math.PI / 2;
    raycastPlane.rotation.x = Math.PI / 2;
    scene.add(raycastPlane);

    // Mouse helper
    let mouseGeometry = new BoxBufferGeometry(1, 1, 1);
    let mouseMaterial = new MeshBasicMaterial({ color: 0xff6e2d, opacity: 0.5 });
    mouseHelper = new Mesh(mouseGeometry, mouseMaterial);
    scene.add(mouseHelper);

    // Skinned mesh
    skeleton = createExample(scene, state);

    // Solver
    solver = new IKSolver();
    targetPoint = new Vector3(0, 10, 0);
}

// function onWindowResize()
// {
//     camera.aspect = window.innerWidth / window.innerHeight;
//     camera.updateProjectionMatrix();
//     effect.setSize(window.innerWidth, window.innerHeight);
// }

function animate()
{
    requestAnimationFrame(animate);
    render();
}

// UPDATE

function render()
{
    updateSize();

    //TODO render for all scenes, adaptive viewport

    // Wiggle bones forward
    if (state.animateBones)
        updateBonesForward(skeleton);

    // Follow cursor
    else if (state.inverseBones)
        updateBonesInverse(skeleton);

    effect.render(scene, camera);
}

function updateSize()
{
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (canvas.width !== width || canvas.height !== height)
    {
        renderer.setSize(width, height, false);
        effect.setSize(width, height);
    }
}

// FORWARD KINEMATICS

// Dummy update function
function updateBonesForward(skl)
{
    time += 0.01;
    for (let i = 0; i < skl.bones.length; i++) {
        skl.bones[i].rotation.z =
            Math.sin(time + i) * 2 / skl.bones.length;
    }
}

// Inverse kinematics

function onMouseMove(event)
{
    // TODO get viewport for all scenes and adapt.
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    // Raycasting
    let raycaster = new Raycaster();
    raycaster.setFromCamera(mouse, camera);
    let intersects = raycaster.intersectObjects([raycastPlane]);
    if (!intersects.length || !intersects[0]) return;

    targetPoint.copy(intersects[0].point);
    mouseHelper.position.copy(targetPoint);
    mouseHasMoved = true;
}

function updateBonesInverse(skl, slv)
{
    // IK
    let chain = skl.bones;
    let constraints = skl.constraints;

    if (mouseHasMoved)
    {
        slv.solve(Solver.FABRIK, chain, targetPoint, 2, constraints, false);
        slv.solve(Solver.CCD, chain, targetPoint, 10, constraints, true);
    }

    mouseHasMoved = false;
}

// Entry
init();
animate();
