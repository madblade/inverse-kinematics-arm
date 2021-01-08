import {
    ArrowHelper,
    Bone,
    CylinderBufferGeometry, DoubleSide,
    Float32BufferAttribute,
    MeshPhongMaterial, PlaneHelper, Skeleton, SkeletonHelper, SkinnedMesh,
    Uint16BufferAttribute,
    Vector3
}                              from 'three';
import { GUI }                 from 'three/examples/jsm/libs/dat.gui.module';
import { IKConstraintsHelper } from './helpers/IKConstraintsHelper';

let gui;
let skeleton;
let boneLength = 5;

function createExample(scene, state)
{
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

    //links — An array of [page: Object] specifying link bones.
    // index — Link bone.
    // limitation — (optional) Rotation axis. Default is undefined.
    // rotationMin — (optional) Rotation minimum limit. Default is undefined.
    // rotationMax — (optional) Rotation maximum limit. Default is undefined.
    // enabled — (optional) Default is true.
    let constraints = {
        effector: 4,
        links: [
            { id: 3, limitation: (new Vector3( 1, 0, 0 )).normalize() },
            // { id: 3 },
            { id: 2, limitation: new Vector3( 0, 0, 1 ) },
            // { id: 2 },
            { id: 1 },
            { id: 0 },
            // { id: 1, limitation: new Vector3( 0, 0, 1 ) },
            // { id: 0, limitation: new Vector3( 1, 0, 0 )}
        ],
        minAngle: 0.0,
        maxAngle: 1.0,
    };

    // Skinned mesh
    let mesh = createSkinnedMesh(sizing);
    scene.add(mesh);

    // Bones
    createBones(sizing, constraints, mesh);

    // Skeleton helper
    let skeletonHelper = new SkeletonHelper(mesh);
    skeletonHelper.material.linewidth = 2;
    scene.add(skeletonHelper);

    // Constraints helper
    createConstraintsHelper(mesh, constraints, scene);

    // GUI
    createGUI(state, mesh);
}

function createBones(sizing, constraints, mesh)
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
    skeleton.constraints = constraints;

    let rootBone = skeleton.bones[0];
    mesh.add(rootBone);
    mesh.bind(skeleton);
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
        color: 0x156289,
        emissive: 0x072534,
        side: DoubleSide,
        flatShading: true
    });

    return new SkinnedMesh(skinnedMeshGeometry, skinnedMeshMaterial);
}

// TODO constraints helper
function createConstraintsHelper(
    mesh, constraints, scene
)
{
    let constraintsHelper = new IKConstraintsHelper(mesh, constraints);
    scene.add(constraintsHelper);
}

function createGUI(state, mesh)
{
    // dat.gui helper
    gui = new GUI();
    let folder = gui.addFolder("General Options");
    let folderIndex = 0;
    folder.add(state, "animateBones");
    folder.__controllers[folderIndex++].name("Animate Bones");
    folder.add(state, "inverseBones");
    folder.__controllers[folderIndex++].name("Inverse Bones");
    folder.add(mesh, "pose");
    folder.__controllers[folderIndex++].name(".pose()");
    let skeleton = mesh.skeleton;
    let skeletonBones = skeleton ? skeleton.bones : [];
    for (let i = 0; i < skeletonBones.length; i++)
    {
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

export { createExample, skeleton }
