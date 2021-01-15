import './style.css';

import content from './content.html';

import {
    AmbientLight,
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
let numberOfDemos = 5;

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
let targetPointHasMoved = [];
let elements = [];

function customizeDemo(states)
{
    // Robotic arm presentation
    states[0].method = METHODS.FORWARD;

    // CCD, unconstrained
    states[1].method = METHODS.CCD;

    // FABRIK, unconstrained
    states[2].method = METHODS.FABRIK;
    // states[2].constrained = true;

    states[3].method = METHODS.CCD;
    states[4].method = METHODS.FABRIK;
}

function wrapDemo(states)
{
    // Bind demo 3 and 4
    mouseHelpers[4].userData.binding = {id: 3, helper: mouseHelpers[3]};
    mouseHelpers[3].userData.binding = {id: 4, helper: mouseHelpers[4]};

    let c1 = scenes[3].userData.camera;
    let v1 = scenes[3].userData.view;
    let c2 = scenes[4].userData.camera;
    let v2 = scenes[4].userData.view;

    let controls1 = new OrbitControls(c1, v2);
    let controls2 = new OrbitControls(c2, v1);
    controls1.enablePan = false;
    controls2.enablePan = false;
}

function createOneConstraintMiddle(state)
{
    let constraints = {
        effector: 4,
        links: [
            { id: 0 },
            { id: 1 },
            { id: 2, limitation: new Vector3( 0, 0, 1 ) },
            { id: 3 },
        ],
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createOneConstraintEnd(state)
{
    let constraints = {
        effector: 4,
        links: [
            { id: 0 },
            { id: 1 },
            { id: 2 },
            { id: 3, limitation: (new Vector3( 1, 0, 0 )) },
        ],
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createTwoConstraints(state)
{

    let constraints = {
        effector: 4,
        links: [
            { id: 0 },
            { id: 1 },
            { id: 2, limitation: new Vector3( 0, 0, 1 ) },
            { id: 3, limitation: (new Vector3( 1, 0, 0 )) },
        ],
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createTwoConstraintsPlusLimitEnd(state)
{
    let constraints = {
        effector: 4,
        links: [
            { id: 0 },
            { id: 1 },
            { id: 2, limitation: new Vector3( 0, 0, 1 ) },
            { id: 3, limitation: (new Vector3( 1, 0, 0 )) },
            { id: 3,
                limitation: new Vector3( 1, 0, 0 ),
                rotationMin: new Vector3(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                rotationMax: new Vector3(Math.PI / 2, Math.PI / 2, Math.PI / 2)
            },
        ],
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createTwoConstraintsPlusLimitMiddle(state)
{

    let constraints = {
        effector: 4,
        links: [
            { id: 0 },
            { id: 1,
                rotationMin: new Vector3(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                rotationMax: new Vector3(Math.PI / 2, Math.PI / 2, Math.PI / 2)
            },
            { id: 2, limitation: new Vector3( 0, 0, 1 ) },
            { id: 3, limitation: (new Vector3( 1, 0, 0 )) },
        ],
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createDefaultConstraints(state)
{
    // constraints format
    // links — An array of [Bones (Object3D)] specifying link bones.
    //  index — Link bone.
    //  limitation — (optional) NORMALIZED rotation axis. Default is undefined.
    //  rotationMin — (optional) Rotation minimum limit. Default is undefined.
    //  rotationMax — (optional) Rotation maximum limit. Default is undefined.
    //  enabled — (optional) Default is true.

    let constraints = {
        effector: 4,
        links: [
            { id: 0 },
            { id: 1 },
            { id: 2 },
            { id: 3 },
        ],
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createDefaultSizing(state)
{
    let boneLength = 5;
    let segmentHeight = boneLength;
    let segmentCount = 4;
    let height = segmentHeight * segmentCount;
    let halfHeight = height * 0.5;
    let sizing = {
        segmentHeight,
        segmentCount,
        height,
        halfHeight
    };
    state.sizing = sizing;
}

function createLongConstraint(state)
{
    let links = new Array(20).fill(0).map((x, i)=>{return {id: i}});
    let constraints = {
        effector: 4,
        links: links,
        minAngle: 0.,
        maxAngle: 1.0
    };
    state.constraints = constraints;
}

function createLongSizing(state)
{
    let boneLength = 1;
    let segmentHeight = boneLength;
    let segmentCount = 20;
    let height = segmentHeight * segmentCount;
    let halfHeight = height * 0.5;
    let sizing = {
        segmentHeight,
        segmentCount,
        height,
        halfHeight
    };
    state.sizing = sizing;
}

function init()
{
    // HTML
    canvas = document.getElementById('c');

    // Renderer
    renderer = new WebGLRenderer({
        canvas: canvas,
        antialias: true,
    });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setClearColor(0x000000, 1);
    effect = new OutlineEffect(renderer, {
        defaultColor: 0xffffff
    });

    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('resize', onWindowResize, false);

    for (let i = 0; i < numberOfDemos; ++i)
    {
        let state = {
            sizing: {},
            method: METHODS.CCD, // none, forward, ccd, fabrik, hybrid
            iterations_ccd: 10, // ignored if !ccd
            iterations_fabrik: 10, // ignored if !fabrik
            iterations_hybrid: [2, 10], // fabrik, then ccd. ignored if !hybrid
            constrained: false,
            constraints: {}
        };
        createDefaultSizing(state);
        createDefaultConstraints(state);
        states.push(state);
    }

    customizeDemo(states);

    for (let i = 0; i < numberOfDemos; ++i)
    {
        // Scene, Camera, Controls, Lights, …
        const view = document.getElementById('demo-' + i);
        if (!view) continue;

        let scene = new Scene();
        scene.background = new Color(0x000000);

        let cameraAspect = 1; // elementWidth / elementHeight;
        let camera = new PerspectiveCamera(45, cameraAspect, 1, 2000);
        // camera.position.z = 30;
        camera.position.z = 40;
        camera.position.y = 30;

        let controls = new OrbitControls(camera, view);
        controls.enablePan = false;
        scene.userData.controls = controls;
        scene.userData.camera = camera;
        scene.userData.view = view;

        let ambient = new AmbientLight(0x666666);
        scene.add(ambient);
        let directionalLight = new DirectionalLight(0x887766);
        directionalLight.position.set(-1, 1, 1).normalize();
        scene.add(directionalLight);

        // Helpers
        let gridHelper = new GridHelper(30, 3, 0x444444, 0x111111);
        gridHelper.position.y = -10;
        scene.add(gridHelper);

        let planeGeometry = new PlaneBufferGeometry(30, 30, 2);
        let planeMaterial = new MeshBasicMaterial({
            color: 0xffff00,
            side: DoubleSide,
            wireframe: true,
            transparent: true,
            opacity: 0
        });
        let raycastPlane = new Mesh(planeGeometry, planeMaterial);
        raycastPlane.rotation.x = Math.PI / 2;
        scene.add(raycastPlane);

        let gridHelper2 = new GridHelper(30, 2);
        scene.add(gridHelper2);

        // Mouse helper
        let relativeMouse = new Vector2();
        let mouseGeometry = new BoxBufferGeometry(1, 1, 1);
        let mouseMaterial = new MeshBasicMaterial({color: 0xff6e2d, opacity: 0.5});
        let mouseHelper = new Mesh(mouseGeometry, mouseMaterial);
        scene.add(mouseHelper);

        // Skinned mesh
        let state = states[i];
        let skeleton = createExample(scene, state);

        // Solver
        let solver = new IKSolver();
        let targetPoint = new Vector3(0, 10, 0);

        // Put into model
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
        targetPointHasMoved.push(false);
    }

    wrapDemo(states);
}

function onWindowResize()
{
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    sizeUpToDate = false;
    updateSize();
    // camera.aspect = window.innerWidth / window.innerHeight;
    // camera.updateProjectionMatrix();
    // effect.setSize(window.innerWidth, window.innerHeight);
}

function animate()
{
    render();
    requestAnimationFrame(animate);
}

// UPDATE

function render()
{
    updateSize();

    renderer.setClearColor( 0x000000 );
    renderer.setScissorTest( false );
    renderer.clear();

    renderer.setClearColor( 0x000000 );
    renderer.setScissorTest( true );

    for (let i = 0; i < numberOfDemos; ++i)
    {
        const scene = scenes[i];
        if (!scene) continue;
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
        else if (state.method !== METHODS.NONE && targetPointHasMoved[i])
            updateBonesInverse(skeleton, solver, targetPoint, state);

        // renderer.render(scene, camera);
        effect.render(scene, camera);
    }

    targetPointHasMoved.fill(false);
    mouseHasMoved = false;
}

let sizeUpToDate = false;
function updateSize()
{
    if (sizeUpToDate) return;
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (canvas.width !== width || canvas.height !== height)
    {
        renderer.setSize(width, height, false);
        effect.setSize(width, height, false);
    }
    sizeUpToDate = true;
}

// FORWARD KINEMATICS

// Dummy update function
function updateBonesForward(skl)
{
    time += 0.01;
    for (let i = 0; i < skl.bones.length; i++) {
        skl.bones[i].rotation.z =
            Math.sin(time + i) * 2 / skl.bones.length;
        skl.bones[i].rotation.y =
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

    for (let i = 0; i < numberOfDemos; ++i)
    {
        const camera = cameras[i];
        if (!camera) continue;
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

        relativeMouse.set(relX, relY);

        // Raycasting
        let raycaster = new Raycaster();
        raycaster.setFromCamera(relativeMouse, camera);
        let intersects = raycaster.intersectObjects([raycastPlane]);
        if (!intersects.length || !intersects[0]) continue;

        targetPoint.copy(intersects[0].point);
        mouseHelper.position.copy(targetPoint);
        targetPointHasMoved[i] = true;

        if (mouseHelper.userData.binding)
        {
            const j = mouseHelper.userData.binding.id;
            const helper = mouseHelper.userData.binding.helper;
            targetPoints[j].copy(targetPoint);
            targetPointHasMoved[j] = true;
            helper.position.copy(targetPoint);
        }

        mouseHasMoved = true;
    }
}

function updateBonesInverse(skl, slv, targetPoint, state)
{
    // IK
    let chain = skl.bones;
    let constraints = skl.constraints;

    if (mouseHasMoved)
    {
        let isConstrained = state.constrained;
        switch (state.method)
        {
            case METHODS.HYBRID:
                const i1 = state.iterations_hybrid[0];
                const i2 = state.iterations_hybrid[1];
                slv.solve(Solver.FABRIK, chain, targetPoint, i1, constraints, false);
                slv.solve(Solver.CCD, chain, targetPoint, i2, constraints, isConstrained);
                break;
            case METHODS.CCD:
                const iterations_ccd = state.iterations_ccd;
                slv.solve(Solver.CCD, chain, targetPoint, iterations_ccd, constraints, isConstrained);
                break;
            case METHODS.FABRIK:
                const iterations_fabrik = state.iterations_fabrik;
                slv.solve(Solver.FABRIK, chain, targetPoint, iterations_fabrik, constraints, isConstrained);
                break;
        }
    }
}


let html = content;
let div = document.createElement('div');
div.innerHTML = html.trim();
document.getElementById('content').appendChild(div);

// Entry
init();
animate();
