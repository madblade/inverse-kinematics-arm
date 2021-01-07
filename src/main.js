import './style.css';

import {
    AmbientLight,
    AxesHelper,
    Bone, BoxBufferGeometry,
    Color,
    CylinderBufferGeometry,
    DirectionalLight,
    DoubleSide,
    Float32BufferAttribute,
    GridHelper,
    Mesh,
    MeshBasicMaterial,
    MeshPhongMaterial,
    PerspectiveCamera,
    Plane,
    PlaneBufferGeometry, Quaternion,
    Raycaster,
    Scene,
    Skeleton,
    SkeletonHelper,
    SkinnedMesh,
    Uint16BufferAttribute,
    Vector2,
    Vector3,
    WebGLRenderer
}                        from "three";
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { OutlineEffect } from 'three/examples/jsm/effects/OutlineEffect';
import Stats             from 'three/examples/jsm/libs/stats.module';
import { GUI }              from 'three/examples/jsm/libs/dat.gui.module';
import { IKSolver, Solver } from './solvers/IKSolver';

let container, stats;
let mesh;
let skeleton;
let gui;
let camera, scene, renderer, effect;
let raycastPlane;
let mouseHelper;
let boneLength = 5;
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

    // Stats, GUI
    gui = new GUI();
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

    // X
    // SKINNED MESH EXAMPLE
    // X

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

    // Skinned mesh
    createSkinnedMesh(sizing);

    // Bones
    createBones(sizing);

    let planeGeometry = new PlaneBufferGeometry(100, 100, 2);
    let planeMaterial = new MeshBasicMaterial({color: 0xffff00, side: DoubleSide});
    raycastPlane = new Mesh(planeGeometry, planeMaterial);

    // Skeleton helper
    let skeletonHelper = new SkeletonHelper(mesh);
    skeletonHelper.material.linewidth = 2;
    scene.add(skeletonHelper);

    // Mouse helper
    let mouseGeometry = new BoxBufferGeometry(1, 1, 1);
    let mouseMaterial = new MeshBasicMaterial({color: 0xff6e2d, opacity: 0.5});
    mouseHelper = new Mesh(mouseGeometry, mouseMaterial);
    scene.add(mouseHelper);

    // dat.gui helper
    let folder = gui.addFolder("General Options");
    let folderIndex = 0;
    folder.add(state, "animateBones");
    folder.__controllers[folderIndex++].name("Animate Bones");
    folder.add(state, "inverseBones");
    folder.__controllers[folderIndex++].name("Inverse Bones");
    folder.add(mesh, "pose");
    folder.__controllers[folderIndex++].name(".pose()");
    let skeletonBones = skeleton ? skeleton.bones : [];
    for (let i = 0; i < skeletonBones.length; i++) {
        let bone = skeletonBones[i];
        folderIndex = 0;
        folder = gui.addFolder("Bone " + i);

        // folder.add(bone.position, 'x', -10 + bone.position.x, 10 + bone.position.x);
        // folder.add(bone.position, 'y', -10 + bone.position.y, 10 + bone.position.y);
        // folder.add(bone.position, 'z', -10 + bone.position.z, 10 + bone.position.z);
        // folder.__controllers[folderIndex++].name("position.x");
        // folder.__controllers[folderIndex++].name("position.y");
        // folder.__controllers[folderIndex++].name("position.z");

        folder.add(bone.rotation, 'x', -Math.PI * 0.5, Math.PI * 0.5, 0.001);
        folder.add(bone.rotation, 'y', -Math.PI * 0.5, Math.PI * 0.5, 0.001);
        folder.add(bone.rotation, 'z', -Math.PI * 0.5, Math.PI * 0.5, 0.001);
        folder.__controllers[folderIndex++].name("rotation.x");
        folder.__controllers[folderIndex++].name("rotation.y");
        folder.__controllers[folderIndex++].name("rotation.z");

        // folder.add(bone.scale, 'x', 0, 2);
        // folder.add(bone.scale, 'y', 0, 2);
        // folder.add(bone.scale, 'z', 0, 2);
        // folder.__controllers[folderIndex++].name("weight.x");
        // folder.__controllers[folderIndex++].name("weight.y");
        // folder.__controllers[folderIndex++].name("weight.z");
    }
}

function createSkinnedMesh(sizing)
{
    let skinnedMeshGeometry = new CylinderBufferGeometry(
        1, // radiusTop
        1, // radiusBottom
        sizing.height, // height
        8, // radiusSegments
        sizing.segmentCount * 3, // heightSegments
        true // openEnded
    );
    let skinnedMeshPositionAttribute = skinnedMeshGeometry.attributes.position;
    let vertex = new Vector3();
    let skinIndices = [];
    let skinWeights = [];
    for (let i = 0; i < skinnedMeshPositionAttribute.count; i++) {
        vertex.fromBufferAttribute(skinnedMeshPositionAttribute, i);
        let y = (vertex.y + sizing.halfHeight);
        let skinIndex = Math.floor(y / sizing.segmentHeight);
        let skinWeight = (y % sizing.segmentHeight) / sizing.segmentHeight;
        skinIndices.push(skinIndex, skinIndex + 1, 0, 0);
        skinWeights.push(1 - skinWeight, skinWeight, 0, 0);
    }
    skinnedMeshGeometry.addAttribute('skinIndex', new Uint16BufferAttribute(skinIndices, 4));
    skinnedMeshGeometry.addAttribute('skinWeight', new Float32BufferAttribute(skinWeights, 4));

    let skinnedMeshMaterial = new MeshPhongMaterial({
        skinning: true,
        color: 0x156289,
        emissive: 0x072534,
        side: DoubleSide,
        flatShading: true
    });

    mesh = new SkinnedMesh(skinnedMeshGeometry, skinnedMeshMaterial);
}

function createBones(sizing)
{
    let bones = [];
    {
        let prevBone = new Bone();
        bones.push(prevBone);
        prevBone.position.y = -sizing.halfHeight;
        for (let i = 0; i < sizing.segmentCount; i++) {
            let bone = new Bone();
            bone.position.y = sizing.segmentHeight;
            bones.push(bone);
            prevBone.add(bone);
            prevBone = bone;
        }
    }

    skeleton = new Skeleton(bones);
    let rootBone = skeleton.bones[0];
    mesh.add(rootBone);
    scene.add(mesh);

    mesh.bind(skeleton);
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

function render()
{
    // Wiggle bones forward
    if (state.animateBones) {
        updateBonesForward();
    }
    // Follow cursor
    else if (state.inverseBones) {
        updateBonesInverse();
    }

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

// Raycasting

let solver = new IKSolver();
function onMouseMove(event)
{
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
}

function updateBonesInverse()
{
    let raycaster = new Raycaster();
    raycaster.setFromCamera(mouse, camera);
    let intersects = raycaster.intersectObjects([raycastPlane]);
    if (!intersects.length || !intersects[0]) return;

    let targetPoint = intersects[0].point;
    mouseHelper.position.copy(targetPoint);

    //
    let chain = skeleton.bones;
    solver.solve(Solver.CCD, chain, targetPoint, 10, null);
}

// Entry

init();
animate();
