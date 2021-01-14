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
let numberOfDemos = 3;
// let container;

// let camera;
// let scene;
// let raycastPlane;
// let mouseHelper;
// let state = {
//     animateBones: false,
//     inverseBones: true
// };
// let skeleton;
// let solver;
// let targetPoint;

const METHODS = {
    NONE: 0,
    FORWARD: 1,
    CCD: 2,
    FABRIK: 3,
    HYBRID: 4
};

let cameras = [];
let scenes = [];
let raycastPlanes = [];
let mouseHelpers = [];
let states = [];
let relativeMice = [];
let skeletons = [];
let solvers = [];
let targetPoints = [];
let elements = [];

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

    window.addEventListener('mousemove', onMouseMove);

    // TODO iterate on scenes, store element
    //  ok?
    for (let i = 0; i < numberOfDemos; ++i)
    {
        // Scene, Camera, Controls, Lights, …
        let state = {
            // animateBones: false,
            // inverseBones: true,
            method: METHODS.CCD, // none, forward, ccd, fabrik, hybrid
            iterations_ccd: 10, // ignored if !ccd
            iterations_fabrik: 10, // ignored if !fabrik
            iterations_hybrid: [2, 10], // fabrik, then ccd. ignored if !hybrid
        };

        const element = document.createElement('div');
        const view = document.getElementById('demo-' + i);
        if (!view) continue;

        let scene = new Scene();
        scene.background = new Color(0x000000);
        // scene.userData.element = element;
        // view.appendChild(element);

        // let elementWidth = window.innerWidth;
        // let elementHeight = window.innerHeight;
        let cameraAspect = 1; // elementWidth / elementHeight;
        let camera = new PerspectiveCamera(45, cameraAspect, 1, 2000);
        camera.position.z = 30;

        let controls = new OrbitControls(camera, view); // renderer.domElement);
        controls.enablePan = false;
        scene.userData.controls = controls;
        // window.addEventListener('resize', onWindowResize, false);

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
        let raycastPlane = new Mesh(planeGeometry, planeMaterial);
        // raycastPlane.rotation.x = Math.PI / 2;
        raycastPlane.rotation.x = Math.PI / 2;
        scene.add(raycastPlane);

        // Mouse helper
        let relativeMouse = new Vector2();
        let mouseGeometry = new BoxBufferGeometry(1, 1, 1);
        let mouseMaterial = new MeshBasicMaterial({color: 0xff6e2d, opacity: 0.5});
        let mouseHelper = new Mesh(mouseGeometry, mouseMaterial);
        scene.add(mouseHelper);

        // Skinned mesh
        let skeleton = createExample(scene, state);

        // Solver
        let solver = new IKSolver();
        let targetPoint = new Vector3(0, 10, 0);

        // Put into model
        // elements.push(element);
        elements.push(view);
        cameras.push(camera);
        scenes.push(scene);
        raycastPlanes.push(raycastPlane);
        mouseHelpers.push(mouseHelper);
        states.push(state);
        relativeMice.push(relativeMouse);
        skeletons.push(skeleton);
        solvers.push(solver);
        targetPoints.push(targetPoint);
    }

    // Customize here
    states[0].method = METHODS.FORWARD;
    states[1].method = METHODS.CCD;
    states[2].method = METHODS.FABRIK;
}

// function onWindowResize()
// {
//     camera.aspect = window.innerWidth / window.innerHeight;
//     camera.updateProjectionMatrix();
//     effect.setSize(window.innerWidth, window.innerHeight);
// }

function animate()
{
    render();
    requestAnimationFrame(animate);
}

// UPDATE

function render()
{
    updateSize();

    // canvas.style.transform = `translateY(${window.scrollY}px)`;

    renderer.setClearColor( 0xffffff );
    renderer.setScissorTest( false );
    renderer.clear();

    renderer.setClearColor( 0x000000 );
    renderer.setScissorTest( true );

    // TODO render for all scenes, adaptive viewport
    //  ok?
    for (let i = 0; i < numberOfDemos; ++i)
    {
        const scene = scenes[i];
        const state = states[i];
        const skeleton = skeletons[i];
        const solver = solvers[i];
        const camera = cameras[i];
        const targetPoint = targetPoints[i];

        const element = elements[i];
        const rect = element.getBoundingClientRect();
        if (rect.bottom < 0 || rect.top > renderer.domElement.clientHeight ||
            rect.right < 0 || rect.left > renderer.domElement.clientWidth)
            continue;
        const width = rect.right - rect.left;
        const height = rect.bottom - rect.top;
        const left = rect.left;
        const bottom = renderer.domElement.clientHeight - rect.bottom;

        renderer.setViewport(left, bottom, width, height);
        renderer.setScissor(left, bottom, width, height);

        // Wiggle bones forward
        if (state.method === METHODS.FORWARD)
            updateBonesForward(skeleton);

        // Follow cursor
        else if (state.method !== METHODS.NONE)
            updateBonesInverse(skeleton, solver, targetPoint, state);

        // renderer.render(scene, camera);
        effect.render(scene, camera);
    }

    mouseHasMoved = false;
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
    let x = event.clientX;
    let y = event.clientY;
    mouse.x = (x / window.innerWidth) * 2 - 1;
    mouse.y = -(y / window.innerHeight) * 2 + 1;

    // TODO get viewport for all scenes and adapt.
    //  ok?
    for (let i = 0; i < numberOfDemos; ++i)
    {
        // const scene = scenes[i];
        // const state = states[i];
        // const skeleton = skeletons[i];
        // const solver = solvers[i];
        const camera = cameras[i];
        const targetPoint = targetPoints[i];
        const mouseHelper = mouseHelpers[i];
        if (!mouseHelper) continue;
        const raycastPlane = raycastPlanes[i];
        const relativeMouse = relativeMice[i];

        const element = elements[i];
        const rect = element.getBoundingClientRect();
        if (rect.bottom < 0 || rect.top > renderer.domElement.clientHeight ||
            rect.right < 0 || rect.left > renderer.domElement.clientWidth)
            continue;
        if (x < rect.left || x > rect.right || y < rect.top || y > rect.bottom ||
            rect.bottom === rect.top || rect.left === rect.right)
            continue;

        const relX = (x - rect.left) / (rect.right - rect.left) * 2 - 1;
        const relY = (rect.top -y) / (rect.bottom - rect.top) * 2 + 1;

        // const width = rect.right - rect.left;
        // const height = rect.bottom - rect.top;
        // const left = rect.left;
        // const bottom = renderer.domElement.clientHeight - rect.bottom;

        // renderer.setViewport(left, bottom, width, height);
        // renderer.setScissor(left, bottom, width, height);
        relativeMouse.set(relX, relY);

        // Raycasting
        let raycaster = new Raycaster();
        raycaster.setFromCamera(relativeMouse, camera);
        let intersects = raycaster.intersectObjects([raycastPlane]);
        if (!intersects.length || !intersects[0]) continue;

        targetPoint.copy(intersects[0].point);
        mouseHelper.position.copy(targetPoint);
    }

    mouseHasMoved = true;
}

function updateBonesInverse(skl, slv, targetPoint, state)
{
    // IK
    let chain = skl.bones;
    let constraints = skl.constraints;

    if (mouseHasMoved)
    {
        switch (state.method)
        {
            case METHODS.HYBRID:
                const i1 = state.iterations_hybrid[0];
                const i2 = state.iterations_hybrid[1];
                slv.solve(Solver.FABRIK, chain, targetPoint, i1, constraints, false);
                slv.solve(Solver.CCD, chain, targetPoint, i2, constraints, true);
                break;
            case METHODS.CCD:
                const iterations_ccd = state.iterations_ccd;
                slv.solve(Solver.CCD, chain, targetPoint, iterations_ccd, constraints, true);
                break;
            case METHODS.FABRIK:
                const iterations_fabrik = state.iterations_fabrik;
                slv.solve(Solver.FABRIK, chain, targetPoint, iterations_fabrik, constraints, true);
                break;
        }
    }
}

// let html = `
// `;
// let div = document.createElement('div');
// div.innerHTML = html.trim();
// document.body.appendChild(div);

// Entry
init();
animate();
