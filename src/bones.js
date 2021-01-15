import {
    Bone, BufferGeometry, Color,
    CylinderBufferGeometry, DoubleSide,
    Float32BufferAttribute, Line, LineBasicMaterial,
    MeshPhongMaterial, Skeleton, SkeletonHelper, SkinnedMesh,
    Uint16BufferAttribute,
    Vector3
} from 'three';
import { GUI }                 from 'three/examples/jsm/libs/dat.gui.module';
import { IKConstraintsHelper } from './helpers/IKConstraintsHelper';

function createExample(scene, state)
{
    let sizing = state.sizing;
    let constraints = state.constraints;

    // Skinned mesh
    let mesh = createSkinnedMesh(sizing);
    scene.add(mesh);

    // Bones
    let skeleton = createBones(sizing, constraints, mesh);

    // Skeleton helper
    let skeletonHelper = new SkeletonHelper(mesh);
    skeletonHelper.material.linewidth = 2;
    scene.add(skeletonHelper);

    // Fabrik helper
    const fabrikHelper = false;
    let m = new LineBasicMaterial({ vertexColors: true });
    let g = new BufferGeometry().setFromPoints(skeleton.chainProxy);
    let colors = [];
    const color1 = new Color(0, 0, 1);
    const color2 = new Color(1, 0.75, 0);
    for (let i = 0; i < skeleton.chainProxy.length; ++i) {
        colors.push(color1.r, color1.g, color1.b);
        colors.push(color2.r, color2.g, color2.b);
    }
    g.setAttribute('color', new Float32BufferAttribute(colors, 3));
    let l = new Line(g, m);
    constraints.line = l;
    if (fabrikHelper)
    {
        scene.add(l);
    }

    // Constraints helper
    createConstraintsHelper(mesh, constraints, scene);

    // GUI
    // createGUI(state, mesh);

    return skeleton;
}

function createBones(sizing, constraints, mesh)
{
    let bones = [];
    let chainProxy = []; // for FABRIK
    let prevBone = new Bone();
    chainProxy.push(new Vector3(0, 0, 0));
    bones.push(prevBone);
    prevBone.position.y = -sizing.halfHeight;
    let fixedBaseLocation = new Vector3();
    fixedBaseLocation.copy(prevBone.position);
    for (let i = 0; i < sizing.segmentCount; i++) {
        let bone = new Bone();
        bone.position.y = sizing.segmentHeight; // relative to parent
        bones.push(bone);
        prevBone.add(bone);
        prevBone = bone;
        chainProxy.push(new Vector3(0, (i + 1) * sizing.segmentHeight, 0));
    }

    let boneLengths = []; // for FABRIK
    for (let i = 0; i < bones.length - 1; ++i)
    {
        boneLengths.push(5);
    }
    boneLengths.push(0);
    constraints.boneLengths = boneLengths;
    constraints.chainProxy = chainProxy;
    constraints.fixedBaseLocation = fixedBaseLocation;

    let skeleton = new Skeleton(bones);
    skeleton.constraints = constraints;
    skeleton.chainProxy = chainProxy;

    let rootBone = skeleton.bones[0];
    mesh.add(rootBone);
    mesh.bind(skeleton);

    return skeleton;
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
    for (let i = 0; i < skinnedMeshPositionAttribute.count; i++)
    {
        vertex.fromBufferAttribute(skinnedMeshPositionAttribute, i);
        let y = (vertex.y + sizing.halfHeight);
        let skinIndex = Math.floor(y / sizing.segmentHeight);
        let skinWeight = (y % sizing.segmentHeight) / sizing.segmentHeight;
        skinIndices.push(skinIndex, skinIndex + 1, 0, 0);
        skinWeights.push(1 - skinWeight, skinWeight, 0, 0);
    }
    skinnedMeshGeometry.setAttribute('skinIndex', new Uint16BufferAttribute(skinIndices, 4));
    skinnedMeshGeometry.setAttribute('skinWeight', new Float32BufferAttribute(skinWeights, 4));

    let skinnedMeshMaterial = new MeshPhongMaterial({
        skinning: true,
        color: 0x894315,
        emissive: 0x341007,
        side: DoubleSide,
        flatShading: true,
        // transparent: true,
        // opacity: 0.5
    });

    return new SkinnedMesh(skinnedMeshGeometry, skinnedMeshMaterial);
}

function createConstraintsHelper(
    mesh, constraints, scene
)
{
    let constraintsHelper = new IKConstraintsHelper(mesh, constraints);
    scene.add(constraintsHelper);
}

function createGUI(state, mesh)
{
    let gui = new GUI();
    let folder = gui.addFolder("General Options");
    let folderIndex = 0;
    // folder.add(state, "animateBones");
    // folder.__controllers[folderIndex++].name("Animate Bones");
    // folder.add(state, "inverseBones");
    // folder.__controllers[folderIndex++].name("Inverse Bones");
    let skeleton = mesh.skeleton;
    let skeletonBones = skeleton ? skeleton.bones : [];
    for (let i = 0; i < skeletonBones.length; i++)
    {
        let bone = skeletonBones[i];
        folderIndex = 0;
        folder = gui.addFolder("Bone " + i);

        folder.add(bone.rotation, 'x', -Math.PI * 0.5, Math.PI * 0.5, 0.001);
        folder.add(bone.rotation, 'y', -Math.PI * 0.5, Math.PI * 0.5, 0.001);
        folder.add(bone.rotation, 'z', -Math.PI * 0.5, Math.PI * 0.5, 0.001);
        folder.__controllers[folderIndex++].name("rotation.x");
        folder.__controllers[folderIndex++].name("rotation.y");
        folder.__controllers[folderIndex++].name("rotation.z");
    }
}

export { createExample }
