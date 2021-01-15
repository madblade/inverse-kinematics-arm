
/**
 * @author madblade
 * @author alansley (Alastair Lansley / Federation University Australia)
 * adapted from https://github.com/FedUni/caliko/
 * MIT license.
 */

import {
    Matrix4,
    Quaternion, Vector3
} from 'three';

function FABRIK()
{
    this.v1 = new Vector3();
    this.v2 = new Vector3();
    this.v3 = new Vector3();
    this.v4 = new Vector3();
    this.v5 = new Vector3();
    this.v6 = new Vector3();
    this.v7 = new Vector3();
    this.v8 = new Vector3();
    this.v9 = new Vector3();
    this.v10 = new Vector3();
    this.v11 = new Vector3();
    this.v12 = new Vector3();

    this.q1 = new Quaternion();
    this.q2 = new Quaternion();

    this.m1 = new Matrix4();
    this.m2 = new Matrix4();

    this.mFixedBaseLocation = new Vector3();

    this.mSolveDistanceThreshold = 0.001; // 1.0;
    this.mMinIterationChange = 0.; //0001;
    this.mFixedBaseMode = true;

    this.ops = 0;
    this.annealOnce = false;
}

FABRIK.prototype.solve = function(
    chain,
    targetPoint,
    iterations,
    constraints,
    activateConstraints
)
{
    let bestSolveDistance = Number.MAX_VALUE;
    let lastPassSolveDistance = Number.MAX_VALUE;
    let currentSolveDistance = Number.MAX_VALUE;

    this.mFixedBaseLocation = constraints.fixedBaseLocation;
    this.ops = 0;

    let matrixWorldInverse = this.m1;
    let root = chain[0].parent;
    if (!root) throw Error('Bone chain must be attached to a SkinnedMesh.');
    matrixWorldInverse.copy(root.matrixWorld).invert();

    let invLinkQ = this.q1;
    let linkPos = this.v1;
    let linkScale = this.v2;

    // Preprocess
    const copyFromChain = true;
    if (copyFromChain)
    {
        const proxies = constraints.chainProxy;
        for (let  i = 0; i < proxies.length; ++i)
        {
            let link = chain[i];
            link.matrixWorld.decompose(linkPos, invLinkQ, linkScale);
            proxies[i].copy(linkPos);
        }
    }

    const annealOnce = this.annealOnce;
    if (annealOnce)
    {
        const proxies = constraints.chainProxy;
        const boneLengths = constraints.boneLengths;
        for (let  i = 0; i < proxies.length; ++i)
        {
            let length = boneLengths[i];
            let currentStart = proxies[i];
            let currentEnd = proxies[i + 1];
            if (i === 0)
            {
                currentStart.set(0, 0, 0);
                currentEnd.set(0, length, 0);
            }
            else if (currentEnd)
            {
                currentEnd.set(0, length + currentStart.y, 0);
            }
        }
    }

    // Allow up to our iteration limit attempts at solving the chain
    let solveDistance;
    let loop;
    for (loop = 0; loop < iterations; ++loop)
    {
        // Solve the chain for this target
        solveDistance = this.iterate(targetPoint, chain, constraints, activateConstraints, loop);

        // Did we solve it for distance? If so, update our best distance and best solution, and also
        // update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run.
        if (solveDistance < bestSolveDistance)
        {
            bestSolveDistance = solveDistance;
            // Note: we should save only the best solution here.
            // bestSolution = this.cloneIkChain();

            // If we are happy that this solution meets our distance requirements then we can exit the loop now
            if (solveDistance <= this.mSolveDistanceThreshold)
            {
                break;
            }
        }
        else // Did not solve to our satisfaction? Okay...
        {
            // Did we grind to a halt? If so break out of loop to set the best distance and solution that we have
            if ( Math.abs(solveDistance - lastPassSolveDistance) < this.mMinIterationChange )
            {
                // console.log('Ground to halt on iteration: ' + loop);
                break;
            }
        }

        // Update the last pass solve distance
        lastPassSolveDistance = solveDistance;

    } // End of loop
    // TODO report iteration number && flops
    // console.log(loop + ' iterations / ' + solveDistance);

    // Update our solve distance and chain configuration to the best solution found
    currentSolveDistance = bestSolveDistance;
    // mChain = bestSolution;

    // Update helper
    let l = constraints.line;
    let p = constraints.chainProxy;
    const positions = l.geometry.attributes.position.array;
    for (let i = 0; i < p.length; ++i)
    {
        positions[3 * i] = p[i].x;
        positions[3 * i + 1] = p[i].y;
        positions[3 * i + 2] = p[i].z;
    }
    l.geometry.attributes.position.needsUpdate = true;

    // Mesh is updated internally by rebuilding the quaternion hierarchy,
    // but we need to do it one final time here after the last backward pass.
    this.recomputeChainQuaternions(chain, constraints);

    return currentSolveDistance;
};

FABRIK.prototype.recomputeChainQuaternions = function(
    chain, constraints
)
{
    let proxies = constraints.chainProxy;
    let currentV1 = new Vector3();
    let currentV2 = new Vector3();
    let currentQ = new Quaternion();
    let lastQ = new Quaternion();
    let t =   new Vector3();
    let s =   new Vector3();
    let nv1 = new Vector3();
    let nv2 = new Vector3();
    for (let i = 0; i < chain.length - 1; ++i)
    {
        let currentBone = chain[i];
        let currentStart = proxies[i];
        let currentEnd = proxies[i + 1];

        // last bone’s unit vector
        if (i === 0)
            currentV1.set(0, 1, 0);
        else
            currentV1.copy(currentV2);

        // current bone’s unit vector
        currentV2.copy(currentEnd);
        currentV2.addScaledVector(currentStart, -1);
        currentV2.normalize();

        // compute quaternion transform from unit vector 1 to unit vector 2
        if (i === 0)
        {
            currentQ.setFromUnitVectors(currentV1, currentV2);
            currentBone.setRotationFromQuaternion(currentQ);
        }
        else
        {
            // work in local world
            let lastBone = chain[i - 1];
            lastBone.matrixWorld.decompose(t, lastQ, s);
            nv1.copy(currentV1);
            nv2.copy(currentV2);
            let angle = nv1.dot(nv2);
            if (angle > 1.) angle = 1.; else if (angle < -1.) angle = -1.;
            angle = Math.acos(angle);
            nv1.crossVectors(nv1, nv2);
            nv1.normalize();

            // transform rotation axis to local world
            nv1.applyQuaternion(lastQ.conjugate());
            currentQ.setFromAxisAngle(nv1, angle);
            currentBone.setRotationFromQuaternion(currentQ);
        }
    }
};

FABRIK.prototype.iterate = function(
    targetPoint, chain, constraints, activateConstraints, passNumber
)
{
    if (chain.length < 1) throw Error('0 bones');

    // fw pass
    this.forward(targetPoint, chain, constraints, activateConstraints, passNumber);

    // bw pass
    this.backward(targetPoint, chain, constraints, activateConstraints, passNumber);

    // compute distance from end to target
    const endIndex = chain.length - 1;
    let endEffector = constraints.chainProxy[endIndex];
    return endEffector.distanceTo(targetPoint);
};

const JointType = {
    NONE: -1,
    BALL: 0,
    LOCAL_HINGE: 1,
    GLOBAL_HINGE: 2,
};

const BaseboneConstraintType3D = {
    NONE: 0,
    LOCAL_HINGE: 1,
    GLOBAL_HINGE: 2,
    // N.B. Rotor unsupported.
}

// chain: contains bones (usually from base to effector)
//      [0] -> base bone (~Object3D)
//      [0-1] -> bone segment No. 1
//      ...
//      [l-1] -> last bone (= effector)
// constraints: object specifying constraints
//      effector: id
//      links: array of bone constraints (usually from effector to base)
//          id: index of bone
//          limitation: Vector3 normalized, HINGE joint rotation axis
//          rotationMin: Vector3 -PI/PI, BALL local Euler rotation min
//          rotationMax: Vector4 -PI/PI, BALL local Euler rotation max
//      minAngle: min step angle
//      maxAngle: max step angle
//      boneLengths: array of bone lengths (last one === effector === 0)
//      chainProxy: (internal) set of 3-vectors, temp skeleton copy.

// ---------- Forward pass from end effector to base -----------
FABRIK.prototype.forward = function(
    targetPoint,
    chain,
    constraints,
    activateConstraints,
    passNumber
)
{
    const thisBonePosition = this.v1;
    const nextBonePosition = this.v3;
    const nextPlusOneBonePosition = this.v2;
    const thisBoneOuterToInnerUV = this.v5;
    const nextBoneOuterToInnerUV = this.v4;
    const relativeHingeRotationAxis = this.v6;
    const newStartLocation = this.v7;

    const localTransform = this.q1;
    const correctionQuaternion = this.q2;

    const boneLengths = constraints.boneLengths;
    const proxy = constraints.chainProxy;
    const chainLength = chain.length - 1; // end bone === end effector in our setup
    const links = constraints.links;

    // Rebuild chain angles from proxy vectors
    if (passNumber >= 0)
        this.recomputeChainQuaternions(chain, constraints);

    // Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)
    for (let loop = chainLength - 1; loop >= 0; --loop)
    {
        // Get the length of the bone we're working on.
        let thisBone = proxy[loop];
        thisBonePosition.copy(thisBone);
        let thisBoneLength = boneLengths[loop];
        let thisBoneJointType;

        // Get constraints.
        const link = links[loop];
        const id = link.id;
        if (id !== loop) throw Error('Please specify all links in the constraints array.');
        const limitation = link.limitation;
        const rotationMin = link.rotationMin;
        const rotationMax = link.rotationMax;
        if (!activateConstraints)
            thisBoneJointType = JointType.NONE;
        else if (limitation)
            thisBoneJointType = JointType.LOCAL_HINGE;
        else if (rotationMin || rotationMax)
            thisBoneJointType = JointType.BALL;
        else
            thisBoneJointType = JointType.BALL;

        // If we are NOT working on the end effector bone
        if (loop !== chainLength - 1)
        {
            // Get the outer-to-inner unit vector of this bone
            // Vec3f thisBoneOuterToInnerUV = thisBone.getDirectionUV().negated();
            nextBonePosition.copy(proxy[loop + 1]);
            thisBoneOuterToInnerUV.copy(nextBonePosition)
                .addScaledVector(thisBonePosition, -1).normalize().negate();

            // Get the joint type for this bone and handle constraints on thisBoneInnerToOuterUV
            if (thisBoneJointType === JointType.BALL)
            {
                // Get the outer-to-inner unit vector of the bone further out
                nextPlusOneBonePosition.copy(proxy[loop + 2]);
                nextBoneOuterToInnerUV.copy(nextPlusOneBonePosition)
                    .addScaledVector(nextBonePosition, -1).normalize().negate();

                // Constrain to relative angle between this bone and the outer bone if required,
                // only supporting a single max angle.
                this.applyBallConstraint(thisBoneOuterToInnerUV, nextBoneOuterToInnerUV,
                    correctionQuaternion, rotationMin, rotationMax);
            }
            else if (thisBoneJointType === JointType.GLOBAL_HINGE)
            {
                // Project this bone outer-to-inner direction onto the hinge rotation axis
                // Note: The returned vector is normalised.
                // thisBoneOuterToInnerUV.projectOntoPlane( hingeRotationAxis );

                // NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions...
                // so we won't.
            }
            else if (thisBoneJointType === JointType.LOCAL_HINGE)
            {
                // Not a basebone? Then construct the local transform...
                if (loop > 0)
                {
                    this.computeLocalTransform(localTransform, loop, chain);

                    // ...don’t forget to transform the hinge rotation axis into the previous bones frame of reference.
                    relativeHingeRotationAxis.copy(limitation).applyQuaternion(localTransform);
                }
                else // ...basebone? Need to construct the transform from the relative constraint UV.
                {
                    // relativeHingeRotationAxis = mBaseboneRelativeConstraintUV;
                    relativeHingeRotationAxis.copy(limitation);
                }

                // Project this bone's outer-to-inner direction onto the plane described by
                // the relative hinge rotation axis
                // Note: The returned vector is normalised.
                thisBoneOuterToInnerUV.projectOnPlane(relativeHingeRotationAxis).normalize();

                // NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions...
                // so we won't.
            }

            // At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
            // so we can set the new inner joint location to be the end joint location of this bone plus the
            // outer-to-inner direction unit vector multiplied by the length of the bone.
            newStartLocation.copy(nextBonePosition).addScaledVector(thisBoneOuterToInnerUV, thisBoneLength);

            // Set the new start joint location for this bone
            thisBone.copy(newStartLocation);

            // If we are not working on the basebone, then we also set the end joint location of
            // the previous bone in the chain (i.e. the bone closer to the base) to be the new
            // start joint location of this bone.
            // [madblade] No need to do that in our implicit bones setup.
        }

        // ######
        // If we ARE working on the end effector bone...
        else if (loop === chainLength - 1)
        {
            // Snap the end effector's end location to the target
            let endEffector = proxy[loop + 1];
            endEffector.copy(targetPoint);

            // Get the UV between the target / end-location (which are now the same) and the start location of this bone
            nextBonePosition.copy(endEffector);
            thisBoneOuterToInnerUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1)
                .normalize().negate();

            // If the end effector is global hinged then we have to snap to it, then keep that
            // resulting outer-to-inner UV in the plane of the hinge rotation axis
            switch (thisBoneJointType)
            {
                case JointType.BALL:
                    // Ball joints do not get constrained on this forward pass
                    break;

                case JointType.GLOBAL_HINGE:
                    // Global hinges get constrained to the hinge rotation axis,
                    // but not the reference axis within the hinge plane
                    // thisBoneOuterToInnerUV.projectOnPlane( hingeRotationAxis );
                    break;

                case JointType.LOCAL_HINGE:
                    // Local hinges get constrained to the hinge rotation axis,
                    // but not the reference axis within the hinge plane

                    // Get rotation transform...
                    this.computeLocalTransform(localTransform, loop, chain);

                    // ...and transform the hinge rotation axis into the previous bones frame of reference.
                    relativeHingeRotationAxis.copy(limitation).applyQuaternion(localTransform);

                    // Project this bone's outer-to-inner direction onto the plane described
                    // by the relative hinge rotation axis
                    // Note: The returned vector is normalised.
                    thisBoneOuterToInnerUV.projectOnPlane(relativeHingeRotationAxis).normalize();
                    break;
            }

            // Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
            // multiplied by the length of the bone.
            newStartLocation.copy(targetPoint).addScaledVector(thisBoneOuterToInnerUV, thisBoneLength);

            // Set the new start joint location for this bone to be new start location...
            thisBone.copy(newStartLocation);

            // ...and set the end joint location of the bone further in to also be at the new start location
            // (if there IS a bone further in - this may be a single bone chain)
            // [madblade] No need to do that in our setup.
        }
    } // End of forward pass
};

// ---------- Backward pass from base to end effector -----------
FABRIK.prototype.backward = function(
    targetPoint,
    chain,
    constraints,
    activateConstraints,
    passNumber
)
{
    const thisBonePosition = this.v1;
    const previousBonePosition = this.v3;
    const thisBoneInnerToOuterUV = this.v5;
    const prevBoneInnerToOuterUV = this.v6;
    const nextBonePosition = this.v7;
    const previousBoneInnerToOuterUV = this.v2;
    const relativeHingeRotationAxis = this.v4;
    const basisVector = this.v8;
    const relativeHingeReferenceAxis = this.v9;
    const newEndLocation = this.v10;

    const localTransform = this.q1;
    const correctionQuaternion = this.q2;

    const proxy = constraints.chainProxy;
    const boneLengths = constraints.boneLengths;
    const chainLength = chain.length - 1; // end bone === end effector!

    // Get base bone constraint.
    let mBaseboneConstraintType = BaseboneConstraintType3D.NONE;
    const base = constraints.links[0];
    if (base.id !== 0) throw Error('Please specify all links in the constraints array.');
    const limitation = base.limitation;
    const rotationMin = base.rotationMin; // unsupported?
    const rotationMax = base.rotationMax;
    if (!activateConstraints)
        mBaseboneConstraintType = BaseboneConstraintType3D.NONE;
    else if (limitation)
        mBaseboneConstraintType = BaseboneConstraintType3D.LOCAL_HINGE;
    else if (rotationMin || rotationMax)
        mBaseboneConstraintType = BaseboneConstraintType3D.NONE;
    else
        mBaseboneConstraintType = BaseboneConstraintType3D.NONE;

    // Rebuild chain angles from proxy vectors
    if (passNumber >= 0)
        this.recomputeChainQuaternions(chain, constraints);

    for (let loop = 0; loop < chainLength; ++loop)
    {
        let thisBone = proxy[loop];
        let thisBoneLength = boneLengths[loop];
        thisBonePosition.copy(thisBone);

        // If we are not working on the basebone
        if (loop !== 0)
        {
            // Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
            let previousBone = proxy[loop - 1];
            previousBonePosition.copy(previousBone);
            let thisEnd = proxy[loop + 1];
            nextBonePosition.copy(thisEnd);
            thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
            prevBoneInnerToOuterUV.copy(thisBonePosition).addScaledVector(previousBonePosition, -1).normalize();

            let jointType;
            const link = constraints.links[loop];
            const id = link.id;
            if (id !== loop) throw Error('Please specify all links in the constraints array.');
            const limitation = link.limitation;
            const rotationMin = link.rotationMin;
            const rotationMax = link.rotationMax;
            if (!activateConstraints) jointType = JointType.NONE;
            else if (limitation) jointType = JointType.LOCAL_HINGE;
            else if (rotationMin || rotationMax) jointType = JointType.BALL;
            else jointType = JointType.BALL;

            // Dealing with a ball joint?
            if (jointType === JointType.BALL)
            {
                // Note: this doesn’t really give good results.
                this.applyBallConstraint(thisBoneInnerToOuterUV, previousBoneInnerToOuterUV,
                    correctionQuaternion, rotationMin, rotationMax);
            }
            else if (jointType === JointType.GLOBAL_HINGE)
            {
                // Unsupported here, but this is pretty straight-forward.
                // Do the same as for local hinges, but without the local transform part.

                // Get the hinge rotation axis and project our inner-to-outer UV onto it
                // thisBoneInnerToOuterUV.projectOnPlane(hingeRotationAxis);
                // (then, apply additional constraints)
            }
            else if (jointType === JointType.LOCAL_HINGE)
            {
                // Transform the hinge rotation axis to be relative to the previous bone in the chain
                let previousBone = proxy[loop - 1];
                previousBoneInnerToOuterUV
                    .copy(thisBonePosition)
                    .addScaledVector(previousBone, -1)
                    .normalize();

                // Construct a rotation matrix based on the previous bone's direction
                this.computeLocalTransform(localTransform, loop, chain);

                // Transform the hinge rotation axis into the previous bone's frame of reference
                relativeHingeRotationAxis.copy(limitation).applyQuaternion(localTransform);

                // Project this bone direction onto the plane described by the hinge rotation axis
                // Note: The returned vector is normalised.
                // thisBoneInnerToOuterUV = thisBoneInnerToOuterUV.projectOntoPlane(relativeHingeRotationAxis);
                thisBoneInnerToOuterUV.projectOnPlane(relativeHingeRotationAxis).normalize();

                // Constrain rotation about reference axis if required
                if (rotationMin !== undefined || rotationMax !== undefined)
                {
                    // Calc. the reference axis in local space
                    basisVector.set(0, 1, 0);
                    relativeHingeReferenceAxis.copy(basisVector).applyQuaternion(localTransform).normalize();

                    // Just apply regular ball constraints.
                    this.applyBallConstraint(thisBoneInnerToOuterUV,
                        relativeHingeReferenceAxis, correctionQuaternion,
                        rotationMin, rotationMax);
                }

            } // End of local hinge section

            // At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
            // so we can set the new inner joint location to be the end joint location of this bone plus the
            // outer-to-inner direction unit vector multiplied by the length of the bone.
            newEndLocation.copy(thisBonePosition).addScaledVector(thisBoneInnerToOuterUV, thisBoneLength);

            // Set the new start joint location for this bone
            thisEnd.copy(newEndLocation);

            // If we are not working on the end effector bone, then we set the start joint location of the next bone in
            // the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
            // [madblade] Still no need to do that for implicit bones.
        }

        // ######
        // If we ARE working on the basebone...
        else
        {
            let thisEnd = proxy[loop + 1];
            nextBonePosition.copy(thisEnd);

            // If the base location is fixed then snap the start location of the basebone back to the fixed base...
            if (this.mFixedBaseMode)
            {
                thisBone.copy(this.mFixedBaseLocation);
            }
            else // ...otherwise project it backwards from the end to the start by its length.
            {
                thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
                thisBone.copy(nextBonePosition).addScaledVector(thisBoneInnerToOuterUV, -thisBoneLength);
            }

            thisBonePosition.copy(thisBone);

            // If the basebone is unconstrained then process it as usual...
            if (mBaseboneConstraintType === BaseboneConstraintType3D.NONE)
            {
                // Set the new end location of this bone, and if there are more bones,
                // then set the start location of the next bone to be the end location of this bone
                thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
                newEndLocation.copy(thisBone).addScaledVector(thisBoneInnerToOuterUV, thisBoneLength);

                thisEnd.copy(newEndLocation);
            }
            else // ...otherwise we must constrain it to the basebone constraint unit vector
            {
                if (mBaseboneConstraintType === BaseboneConstraintType3D.GLOBAL_HINGE)
                {
                    // Again, this is the same as for local hinges, feel free to implement it without .
                    // hingeRotationAxis  =  getRotationAxis()
                    // cwConstraintDegs   = -clockwiseConstraintDegs // Clockwise rotation is negative
                    // acwConstraintDegs  =  anticlockwiseConstraintDegs

                    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
                    // thisBoneInnerToOuterUV.projectOnPlane(hingeRotationAxis);

                    // … then apply constraints about the reference axis.
                    // (and don’t forget to apply changes to the bone)
                }
                else if (mBaseboneConstraintType === BaseboneConstraintType3D.LOCAL_HINGE)
                {
                    let hingeRotationAxis = limitation;
                    //      Basebone relative constraint is our hinge rotation axis.

                    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
                    // Vec3f thisBoneInnerToOuterUV = thisBone.getDirectionUV().projectOntoPlane(hingeRotationAxis);
                    thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
                    thisBoneInnerToOuterUV.projectOnPlane(hingeRotationAxis).normalize();

                    if (rotationMin || rotationMax)
                    {
                        // Working in absolute space, I’m not really sure these are well-defined.
                        basisVector.set(0, 1, 0);
                        if (limitation.distanceTo(basisVector) < 0.001) basisVector.set(1, 0, 0);
                        relativeHingeReferenceAxis.copy(basisVector).cross(hingeRotationAxis).normalize();

                        // Just apply regular ball constraints.
                        // Constraints are then different from CCD constraints.
                        // Letting this as is for the demo.
                        this.applyBallConstraint(thisBoneInnerToOuterUV,
                            relativeHingeReferenceAxis, correctionQuaternion,
                            rotationMin, rotationMax);
                    }

                    // Calc and set the end location of this bone
                    newEndLocation.copy(thisBone).addScaledVector(thisBoneInnerToOuterUV, thisBoneLength);
                    thisEnd.copy(newEndLocation);

                    // Also, set the start location of the next bone to be the end location of this bone
                    // [madblade] (unnecessary)
                }

            } // End of basebone constraint handling section

        } // End of basebone handling section

    } // End of backward-pass loop over all bones
};

FABRIK.prototype.applyBallConstraint = function(
    thisBoneUV, otherBoneUV,
    correctionQuaternion,
    rotationMin, rotationMax
)
{
    let angleBetween = Math.acos(otherBoneUV.dot(thisBoneUV));
    let maxAngle = Math.PI;
    if (rotationMin) maxAngle = -Math.max(rotationMin.x, rotationMin.y, rotationMin.z);
    if (rotationMax) maxAngle = Math.min(maxAngle, rotationMax.x, rotationMax.y, rotationMax.z);
    if (Math.abs(angleBetween) > maxAngle)
    {
        let correctionAxis = new Vector3();
        correctionAxis.crossVectors(otherBoneUV, thisBoneUV).normalize();
        correctionQuaternion.setFromAxisAngle(correctionAxis,
            -Math.sign(angleBetween) * (Math.abs(angleBetween) - maxAngle));
        thisBoneUV.applyQuaternion(correctionQuaternion);
    }
};

FABRIK.prototype.computeLocalTransform = function(localTransform, loop, chain)
{
    let matrixWorldInverse = this.m1;
    let boneMatrix = this.m2;
    const t = this.v11;
    const s = this.v12;
    boneMatrix.multiplyMatrices(matrixWorldInverse, chain[loop - 1].matrixWorld);
    boneMatrix.decompose(t, localTransform, s);
};

export { FABRIK };
