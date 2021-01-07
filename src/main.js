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
import { GUI }           from 'three/examples/jsm/libs/dat.gui.module';

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

init();
animate();

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

    let bones = skeleton.bones;
    let q = new Quaternion();
    let targetVec = new Vector3();
    let effectorPos = new Vector3();
    let effectorVec = new Vector3();
    let linkPos = new Vector3();
    let invLinkQ = new Quaternion();
    let linkScale = new Vector3();
    let axis = new Vector3();
    let vector = new Vector3();

    let math = Math; // for reference overhead reduction in loop
    let ik = {
        effector: 4,
        links: [
            { id: 3, limitation: new Vector3( 1, 0, 0 ) },
            { id: 2 },
            { id: 1 },
            { id: 0 }
        ],
        iteration: 10,
        minAngle: 0.0,
        maxAngle: 1.0,
    };

    let effector = bones[ik.effector];
    // let target = bones[ik.target];
    // don't use getWorldPosition() here for the performance
    // because it calls updateMatrixWorld( true ) inside.
    // targetPos.setFromMatrixPosition(target.matrixWorld);
    let targetPos = targetPoint;
    let links = ik.links;
    let iteration = ik.iteration !== undefined ? ik.iteration : 1;
    for (let j = 0; j < iteration; j++)
    {
        let rotated = false;
        for (let k = 0, kl = links.length; k < kl; k++)
        {
            let link = bones[links[k].id];
            // Skip this link and following links.
            // (this skip can be used for performance optimization)
            if (links[k].enabled === false) break;

            let limitation = links[k].limitation;
            let rotationMin = links[k].rotationMin;
            let rotationMax = links[k].rotationMax;

            // Don't use getWorldPosition/Quaternion() here for the performance
            // because they call updateMatrixWorld(true) inside.
            link.matrixWorld.decompose(linkPos, invLinkQ, linkScale);
            invLinkQ.inverse();
            effectorPos.setFromMatrixPosition(effector.matrixWorld);

            // Work in link world.
            effectorVec.subVectors(effectorPos, linkPos);
            effectorVec.applyQuaternion(invLinkQ);
            effectorVec.normalize();
            targetVec.subVectors(targetPos, linkPos);
            targetVec.applyQuaternion(invLinkQ);
            targetVec.normalize();
            let angle = targetVec.dot(effectorVec);
            if (angle > 1.0) angle = 1.0;
            else if (angle < -1.0) angle = -1.0;
            angle = math.acos(angle);

            // Skip if changing angle is too small to prevent bone vibration.
            if (angle < 1e-5) continue;
            if (ik.minAngle !== undefined && angle < ik.minAngle) angle = ik.minAngle;
            if (ik.maxAngle !== undefined && angle > ik.maxAngle) angle = ik.maxAngle;
            axis.crossVectors(effectorVec, targetVec);
            axis.normalize();

            // Apply.
            q.setFromAxisAngle(axis, angle);
            let qq = new Quaternion();
            qq.copy(link.quaternion).multiply(q);
            link.quaternion.multiply(q);
            // TODO think about this slerp function.
            // link.quaternion.slerp(qq, 0.05);

            if (limitation !== undefined) {
                // TODO reconsider limitation specification
                let c = link.quaternion.w;
                if (c > 1.0) c = 1.0;
                let c2 = math.sqrt(1 - c * c);
                link.quaternion.set(limitation.x * c2, limitation.y * c2, limitation.z * c2, c);
            }

            {
                // TODO softify at min/max
                if (rotationMin !== undefined)
                    link.rotation.setFromVector3(link.rotation.toVector3(vector).max(rotationMin));
                if (rotationMax !== undefined)
                    link.rotation.setFromVector3(link.rotation.toVector3(vector).min(rotationMax));
            }

            link.updateMatrixWorld(true);
            rotated = true;
        }

        if (!rotated) break;
    }
}
