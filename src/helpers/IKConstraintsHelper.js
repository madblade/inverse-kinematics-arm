import {
    ArrowHelper,
    BufferGeometry,
    Color, DoubleSide,
    Float32BufferAttribute,
    LineBasicMaterial, LineSegments,
    Matrix4,
    Mesh, MeshBasicMaterial, Object3D, PlaneBufferGeometry, Quaternion,
    Vector3
} from 'three';

const _upVector = new Vector3();
const _boneMatrix = new Matrix4();
const _matrixWorldInv = new Matrix4();

const _parentPosition = new Vector3();
const _childPosition = new Vector3();
const _position = new Vector3();
const _quaternion = new Quaternion();
const _quaternionConstraint = new Quaternion();
const _defaultUnitVector = new Vector3(0, 0, 1);
const _scale = new Vector3();
const _localLimitation = new Vector3();

class IKConstraintsHelper extends LineSegments
{
    constructor(object, constraints)
    {
        const bones = getBoneList(object);
        const geometry = new BufferGeometry();
        const vertices = [];
        const colors = [];
        const color1 = new Color(1, 0, 0);
        const color2 = new Color(1, 0, 0);
        const color3 = new Color(1, 0.64, 0);

        for (let i = 0; i < bones.length; ++i) {
            const bone = bones[i];
            if (bone.parent && bone.parent.isBone) {
                vertices.push(0, 0, 0);
                vertices.push(0, 0, 0);
                colors.push(color1.r, color1.g, color1.b);
                colors.push(color2.r, color2.g, color2.b);
            }
        }

        geometry.setAttribute('position', new Float32BufferAttribute(vertices, 3));
        geometry.setAttribute('color', new Float32BufferAttribute(colors, 3));

        const material = new LineBasicMaterial({
            vertexColors: true,
            depthTest: false,
            depthWrite: false,
            toneMapped: false,
            transparent: true
        });

        super(geometry, material);

        this.type = 'IKConstraintsHelper';
        this.isSkeletonHelper = true;

        this.root = object;
        this.bones = bones;
        this.matrix = object.matrixWorld;
        this.matrixAutoUpdate = false;
        this.constraints = constraints;
        this.boneConstraints = [];
        const linkConstraints = constraints.links;
        const nbConstraints = linkConstraints.length;
        for (let i = 0; i < nbConstraints; ++i) {
            let l = linkConstraints[i];
            if (l.enabled === false || !l.limitation) continue;
            // let link = bones[l.id];
            // let limitation = l.limitation;
            this.boneConstraints[l.id] = {
                limitation: l.limitation,
                minAngle: l.minAngle,
                maxAngle: l.maxAngle
            };
        }
        const gizmoMaterialY = new MeshBasicMaterial({
            depthTest: false,
            depthWrite: false,
            transparent: true,
            opacity: 0.8,
            side: DoubleSide,
            fog: false,
            toneMapped: false,
            color: 0xffff00
        });
        const gizmoMaterialO = gizmoMaterialY.clone();
        gizmoMaterialO.color.set(0xffa500);
        const planeHelpers = new Object3D();
        this.planeModel = [];
        const planeSize = 1.6;
        for (let i = 0; i < nbConstraints; ++i)
        {
            let l = linkConstraints[i];
            if (l.enabled === false || !l.limitation) continue;
            let geometry1 = new PlaneBufferGeometry(planeSize, planeSize);
            let geometry2 = new PlaneBufferGeometry(planeSize, planeSize);
            let plane1 = new Mesh(geometry1, gizmoMaterialY);
            let plane2 = new Mesh(geometry2, gizmoMaterialO);
            this.planeModel[l.id] = [plane1, plane2];
            planeHelpers.add(plane1);
            planeHelpers.add(plane2);
        }
        // this.planeHelpers = planeHelpers;
        this.add(planeHelpers);
    }

    updateMatrixWorld(force)
    {
        const bones = this.bones;
        const geometry = this.geometry;
        const position = geometry.getAttribute('position');
        const constraints = this.boneConstraints;
        const planes = this.planeModel;

        _matrixWorldInv.copy(this.root.matrixWorld).invert();

        for (let i = 0, j = 0; i < bones.length; ++i)
        {
            const bone = bones[i];
            const constraint = constraints[i];
            if (bone.parent && bone.parent.isBone && constraint)
            {
                const limitation = constraint.limitation;
                const dualPlanes = planes[i];
                const plane1 = dualPlanes[0];
                const plane2 = dualPlanes[1];

                _boneMatrix.multiplyMatrices(_matrixWorldInv, bone.parent.matrixWorld);
                _parentPosition.setFromMatrixPosition(_boneMatrix);

                _boneMatrix.decompose(_position, _quaternion, _scale);
                // compute joint constraint orientation from parent quaternion & axis
                _quaternionConstraint.setFromUnitVectors(_defaultUnitVector, limitation);

                _quaternionConstraint.premultiply(_quaternion);
                plane1.quaternion.copy(_quaternionConstraint);
                plane2.quaternion.copy(_quaternionConstraint);

                _boneMatrix.multiplyMatrices(_matrixWorldInv, bone.matrixWorld);
                _upVector.setFromMatrixPosition(_boneMatrix);
                position.setXYZ(j, _upVector.x, _upVector.y, _upVector.z);
                _childPosition.copy(_upVector);

                // replace joint constraints and translate
                _localLimitation.copy(limitation);
                _localLimitation.applyQuaternion(_quaternion);
                plane1.position.copy(_upVector).addScaledVector(_localLimitation, -0.2);
                plane2.position.copy(_upVector).addScaledVector(_localLimitation, 0.2);

                // get joint orientation
                _quaternionConstraint.copy(_quaternion).conjugate();
                _boneMatrix.decompose(_position, _quaternion, _scale);
                _upVector.subVectors(_parentPosition, _upVector); // child to parent
                _upVector.cross(_localLimitation);
                _upVector.normalize();
                _upVector
                    .applyQuaternion(_quaternionConstraint)
                    .applyQuaternion(_quaternion);
                _upVector.normalize();

                position.setXYZ(j + 1,
                    _childPosition.x + _upVector.x,
                    _childPosition.y + _upVector.y,
                    _childPosition.z + _upVector.z);

                j += 2;
            }
            else if (bone.parent && bone.parent.isBone)
            {
                _boneMatrix.multiplyMatrices(_matrixWorldInv, bone.parent.matrixWorld);
                _parentPosition.setFromMatrixPosition(_boneMatrix);

                _boneMatrix.decompose(_position, _quaternion, _scale);
                _boneMatrix.multiplyMatrices(_matrixWorldInv, bone.matrixWorld);
                _upVector.setFromMatrixPosition(_boneMatrix);
                position.setXYZ(j, _upVector.x, _upVector.y, _upVector.z);
                _childPosition.copy(_upVector);

                _quaternionConstraint.copy(_quaternion).conjugate(); // parent quaternion
                _boneMatrix.decompose(_position, _quaternion, _scale);
                _upVector.subVectors(_parentPosition, _upVector);
                _upVector.normalize();
                _upVector.applyQuaternion(_quaternion);
                _upVector.normalize();

                position.setXYZ(j + 1,
                    _childPosition.x + _upVector.x,
                    _childPosition.y + _upVector.y,
                    _childPosition.z + _upVector.z);

                j += 2;
            }
        }

        geometry.getAttribute( 'position' ).needsUpdate = true;
        super.updateMatrixWorld( force );
    }
}


function getBoneList(object)
{
    const boneList = [];
    if (object && object.isBone)
        boneList.push(object);
    for (let i = 0; i < object.children.length; ++i)
        boneList.push.apply(boneList, getBoneList(object.children[i]));

    return boneList;
}

export { IKConstraintsHelper };
