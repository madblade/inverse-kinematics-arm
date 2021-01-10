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
import Stats                from 'three/examples/jsm/libs/stats.module';
import { IKSolver, Solver }        from './solvers/IKSolver';
import { createExample, skeleton } from './bones';

let container, stats;
let camera, scene, renderer, effect;
let raycastPlane;
let mouseHelper;
let state = {
    animateBones: false,
    inverseBones: true
};
let mouse = new Vector2();

function init()
{
    // HTML
    container = document.createElement('div');
    document.body.appendChild(container);

    // Renderer
    renderer = new WebGLRenderer({antialias: true});
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    container.appendChild(renderer.domElement);
    effect = new OutlineEffect(renderer);

    // Stats
    stats = new Stats();
    container.appendChild(stats.dom);

    // Scene, Camera, Controls, Lights
    scene = new Scene();
    scene.background = new Color(0xffffff);
    camera = new PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 2000);
    camera.position.z = 30;
    let controls = new OrbitControls(camera, renderer.domElement);
    controls.enablePan = false;
    window.addEventListener('resize', onWindowResize, false);
    window.addEventListener('mousemove', onMouseMove);

    let ambient = new AmbientLight(0x666666);
    scene.add(ambient);
    let directionalLight = new DirectionalLight(0x887766);
    directionalLight.position.set(-1, 1, 1).normalize();
    scene.add(directionalLight);

    // Helpers
    // var gridHelper = new PolarGridHelper(30, 10);
    let gridHelper = new GridHelper(30, 10);
    gridHelper.position.y = -10;
    scene.add(gridHelper);
    let axesHelper = new AxesHelper(5);
    axesHelper.position.y = -10;
    axesHelper.position.x = 10;
    axesHelper.position.z = 10;
    scene.add(axesHelper);

    let planeGeometry = new PlaneBufferGeometry(100, 100, 2);
    let planeMaterial = new MeshBasicMaterial({color: 0xffff00, side: DoubleSide});
    raycastPlane = new Mesh(planeGeometry, planeMaterial);
    raycastPlane.rotation.x = Math.PI / 2;

    // Mouse helper
    let mouseGeometry = new BoxBufferGeometry(1, 1, 1);
    let mouseMaterial = new MeshBasicMaterial({color: 0xff6e2d, opacity: 0.5});
    mouseHelper = new Mesh(mouseGeometry, mouseMaterial);
    scene.add(mouseHelper);

    // X
    // SKINNED MESH EXAMPLE
    // X
    createExample(scene, state);
}

function onWindowResize()
{
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    effect.setSize(window.innerWidth, window.innerHeight);
}

function animate()
{
    requestAnimationFrame(animate);
    stats.begin();
    render();
    stats.end();
}

// UPDATE

function render()
{
    // Wiggle bones forward
    if (state.animateBones)
        updateBonesForward();

    // Follow cursor
    else if (state.inverseBones)
        updateBonesInverse();

    effect.render(scene, camera);
}

// FORWARD KINEMATICS

// Dummy update function
let time = 0;
function updateBonesForward()
{
    time += 0.01;
    for (let i = 0; i < skeleton.bones.length; i++) {
        skeleton.bones[i].rotation.z =
            Math.sin(time + i) * 2 / skeleton.bones.length;
    }
}

// INVERSE KINEMATICS

let targetPoint = new Vector3(0, 10, 0);
function onMouseMove(event)
{
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    // Raycasting
    let raycaster = new Raycaster();
    raycaster.setFromCamera(mouse, camera);
    let intersects = raycaster.intersectObjects([raycastPlane]);
    if (!intersects.length || !intersects[0]) return;

    targetPoint.copy(intersects[0].point);
    mouseHelper.position.copy(targetPoint);
}

let solver = new IKSolver();
function updateBonesInverse()
{
    // IK
    let chain = skeleton.bones;
    let constraints = skeleton.constraints;
    solver.solve(Solver.FABRIK, chain, targetPoint, 10, constraints);
}

// Entry

init();
animate();
