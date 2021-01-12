
// from https://github.com/jsantell/THREE.IK/blob/master/src/IKChain.js
// https://github.com/jsantell/THREE.IK/blob/master/src/IK.js
// https://github.com/jsantell/THREE.IK/issues/2
// https://github.com/jsantell/THREE.IK/issues/3
// https://github.com/jsantell/THREE.IK/issues/4
// http://number-none.com/product/IK%20with%20Quaternion%20Joint%20Limits/index.html
//

// from https://github.com/FedUni/caliko/

import {
    Quaternion, Vector3
} from 'three';

function FABRIK()
{
    this.v1 = new Vector3();
    this.v2 = new Vector3();
    this.q1 = new Quaternion();
    this.v3 = new Vector3();
    this.v4 = new Vector3();
    this.q2 = new Quaternion();

    this.v5 = new Vector3();
    this.v6 = new Vector3();

    this.v7 = new Vector3();
    this.q3 = new Quaternion();
    this.v8 = new Vector3();

    this.mFixedBaseLocation = new Vector3();
}

const mSolveDistanceThreshold = 1.0;
const mMinIterationChange = 0.01;
const mFixedBaseMode = true;

// TODO compute flops
FABRIK.prototype.solve = function(
    chain,
    targetPoint,
    iterations,
    constraints
)
{
    let bestSolveDistance = Number.MAX_VALUE;
    let lastPassSolveDistance = Number.MAX_VALUE;
    let currentSolveDistance = Number.MAX_VALUE;

    // this.mFixedBaseLocation = chain[0].position;
    this.mFixedBaseLocation = constraints.fixedBaseLocation;

    // Allow up to our iteration limit attempts at solving the chain
    let solveDistance;
    for (let loop = 0; loop < iterations; ++loop)
    {
        // Solve the chain for this target
        for (let i = 0; i < 10; ++i) // iterate a few times for now
        solveDistance = this.iterate(targetPoint, chain, constraints);

        // Did we solve it for distance? If so, update our best distance and best solution, and also
        // update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run.
        if (solveDistance < bestSolveDistance)
        {
            bestSolveDistance = solveDistance;
            // bestSolution = this.cloneIkChain();

            // If we are happy that this solution meets our distance requirements then we can exit the loop now
            if (solveDistance <= mSolveDistanceThreshold)
            {
                break;
            }
        }
        else // Did not solve to our satisfaction? Okay...
        {
            // Did we grind to a halt? If so break out of loop to set the best distance and solution that we have
            if ( Math.abs(solveDistance - lastPassSolveDistance) < mMinIterationChange )
            {
                //System.out.println("Ground to halt on iteration: " + loop);
                break;
            }
        }

        // Update the last pass solve distance
        lastPassSolveDistance = solveDistance;

    } // End of loop

    // Update our solve distance and chain configuration to the best solution found
    currentSolveDistance = bestSolveDistance;
    // mChain = bestSolution;

    // Update our base and target locations
    // mLastBaseLocation.set( getBaseLocation() );
    // mLastTargetLocation.set(newTarget);

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

    // Update mesh
    let proxies = constraints.chainProxy;
    let currentV1 = new Vector3();
    let currentV2 = new Vector3();
    let currentQ = new Quaternion();
    let lastQ = new Quaternion();
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
        currentV2.copy(currentEnd).addScaledVector(currentStart, -1).normalize();

        if (i === 0)
        {
            currentQ.setFromUnitVectors(currentV1, currentV2);
            currentBone.setRotationFromQuaternion(currentQ);

            // let nv1 = new Vector3();
            // let nv2 = new Vector3();
            // nv1.copy(currentV1);
            // nv2.copy(currentV2);
            // let angle = nv1.dot(nv2); if (angle > 1.) angle = 1.; else if (angle < -1.) angle = -1.;
            // angle = Math.acos(angle);
            // nv1.crossVectors(nv1, nv2);
            // nv1.normalize();
            // currentQ.setFromAxisAngle(nv1, angle);
            // currentBone.setRotationFromQuaternion(currentQ);
        }
        else
        {
            let lastBone = chain[i - 1];
            let t = new Vector3();
            let s = new Vector3();
            lastBone.matrixWorld.decompose(t, lastQ, s);
            let nv1 = new Vector3();
            let nv2 = new Vector3();
            nv1.copy(currentV1);
            nv2.copy(currentV2);
            let angle = nv1.dot(nv2); if (angle > 1.) angle = 1.; else if (angle < -1.) angle = -1.;
            angle = Math.acos(angle);
            nv1.crossVectors(nv1, nv2);
            nv1.normalize()
            nv1.applyQuaternion(lastQ.conjugate());
            currentQ.setFromAxisAngle(nv1, angle);
            // currentQ.setFromUnitVectors(nv1, nv2);
            // currentBone.setRotationFromQuaternion(lastQ);
            // currentBone.applyQuaternion(currentQ)
            currentBone.setRotationFromQuaternion(currentQ);
        }
    }

    return currentSolveDistance;
};

FABRIK.prototype.iterate = function(
    targetPoint, chain, constraints
)
{
    if (chain.length < 1) throw Error('0 bones');

    this.forward(targetPoint, chain, constraints);

    this.backward(targetPoint, chain, constraints);

    // TODO compute distance from end to target
    let distance = 0;
    return distance;
};

const JointType = {
    BALL: 0,
    LOCAL_HINGE: 1,
    GLOBAL_HINGE: 2,
};

const BaseboneConstraintType3D = {
    NONE: 0,
    LOCAL_HINGE: 1,
    GLOBAL_HINGE: 2,
    LOCAL_ROTOR: 3,
    GLOBAL_ROTOR: 4,
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
    constraints
)
{
    const thisBonePosition = this.v1;
    // const thisBoneScale = this.v2;
    // const thisBoneQuaternion= this.q1;
    const nextBonePosition = this.v3;
    // const nextBoneScale = this.v4;
    // const nextBoneQuaternion = this.q2;
    const thisBoneOuterToInnerUV = this.v5;

    const boneLengths = constraints.boneLengths;
    const proxy = constraints.chainProxy;

    const chainLength = chain.length - 1; // end bone === end effector in our setup
    const links = constraints.links;

    // Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)
    for (let loop = chainLength - 1; loop >= 0; --loop)
    {
        // Get the length of the bone we're working on
        // let thisBone = chain[loop];
        let thisBone = proxy[loop];
        thisBonePosition.copy(thisBone);
        // thisBone.matrixWorld.decompose(thisBonePosition, thisBoneQuaternion, thisBoneScale);
        // let thisBoneLength = thisBone.length();
        let thisBoneLength = boneLengths[loop];
        // FabrikJoint3D thisBoneJoint = thisBone.getJoint();
        // JointType thisBoneJointType = thisBone.getJointType();
        let thisBoneJointType;

        const link = links[loop];
        const id = link.id;
        if (id !== loop) throw Error('Please specify all links in the constraints array.');
        const limitation = link.limitation;
        const rotationMin = link.rotationMin;
        const rotationMax = link.rotationMax;
        if (limitation)
            thisBoneJointType = JointType.LOCAL_HINGE;
        else if (rotationMin || rotationMax)
            thisBoneJointType = JointType.BALL;
        else
            thisBoneJointType = JointType.BALL;

        // If we are NOT working on the end effector bone
        if (loop !== chainLength - 1)
        {
            // Get the outer-to-inner unit vector of the bone further out
            // Vec3f outerBoneOuterToInnerUV = mChain.get(loop+1).getDirectionUV().negated();

            // Get the outer-to-inner unit vector of this bone
            // Vec3f thisBoneOuterToInnerUV = thisBone.getDirectionUV().negated();
            let nextBone = chain[loop + 1];
            // nextBone.matrixWorld.decompose(nextBonePosition, nextBoneQuaternion, nextBoneScale);
            nextBone = proxy[loop + 1];
            nextBonePosition.copy(nextBone);
            thisBoneOuterToInnerUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize().negate();

            // Get the joint type for this bone and handle constraints on thisBoneInnerToOuterUV
            if (thisBoneJointType === JointType.BALL)
            {
                // Constrain to relative angle between this bone and the outer bone if required
                // float angleBetweenDegs    = Vec3f.getAngleBetweenDegs(outerBoneOuterToInnerUV, thisBoneOuterToInnerUV);
                // float constraintAngleDegs = thisBoneJoint.getBallJointConstraintDegs();
                // if (angleBetweenDegs > constraintAngleDegs)
                // {
                //     thisBoneOuterToInnerUV = Vec3f.getAngleLimitedUnitVectorDegs(thisBoneOuterToInnerUV, outerBoneOuterToInnerUV, constraintAngleDegs);
                // }
            }
            else if (thisBoneJointType === JointType.GLOBAL_HINGE)
            {
                // Project this bone outer-to-inner direction onto the hinge rotation axis
                // Note: The returned vector is normalised.
                // thisBoneOuterToInnerUV = thisBoneOuterToInnerUV.projectOntoPlane( thisBoneJoint.getHingeRotationAxis() );

                // NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.
            }
            else if (thisBoneJointType === JointType.LOCAL_HINGE)
            {
                // Not a basebone? Then construct a rotation matrix based on the previous bones inner-to-to-inner direction...
                // Mat3f m;
                // Vec3f relativeHingeRotationAxis;
                let relativeHingeRotationAxis = new Vector3();
                if (loop > 0)
                {
                    let previousBone = proxy[loop - 1];
                    let previousBoneInnerToOuterUV = new Vector3();
                    previousBoneInnerToOuterUV.copy(thisBonePosition).addScaledVector(previousBone, -1).normalize();
                    let basisVector = new Vector3(0, 1, 0); // TODO debug
                    let q = new Quaternion();
                    q.setFromUnitVectors(basisVector, previousBoneInnerToOuterUV);
                    relativeHingeRotationAxis.copy(limitation).applyQuaternion(q);
                //     m = Mat3f.createRotationMatrix( mChain.get(loop-1).getDirectionUV() );
                //     relativeHingeRotationAxis = m.times( thisBoneJoint.getHingeRotationAxis() ).normalise();
                }
                else // ...basebone? Need to construct matrix from the relative constraint UV.
                {
                    // [madblade]: I think there’s no need to do that
                    // relativeHingeRotationAxis = mBaseboneRelativeConstraintUV;
                }

                // ...and transform the hinge rotation axis into the previous bones frame of reference.
                //Vec3f

                // Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
                // Note: The returned vector is normalised.
                // thisBoneOuterToInnerUV = thisBoneOuterToInnerUV.projectOntoPlane(relativeHingeRotationAxis);
                thisBoneOuterToInnerUV.projectOnPlane(relativeHingeRotationAxis);

                // NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.
            }

            // At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
            // so we can set the new inner joint location to be the end joint location of this bone plus the
            // outer-to-inner direction unit vector multiplied by the length of the bone.
            // Vec3f newStartLocation = thisBone.getEndLocation().plus( thisBoneOuterToInnerUV.times(thisBoneLength) );
            let newStartLocation = new Vector3();
            newStartLocation.copy(nextBonePosition).addScaledVector(thisBoneOuterToInnerUV, thisBoneLength);

            // Set the new start joint location for this bone
            // thisBone.setStartLocation(newStartLocation);
            // thisBone.position.copy(newStartLocation);
            thisBone.copy(newStartLocation);

            // [madblade] I think there’s no need to do the following.
            // If we are not working on the basebone, then we also set the end joint location of
            // the previous bone in the chain (i.e. the bone closer to the base) to be the new
            // start joint location of this bone.
            // if (loop > 0)
            // {
            //     mChain.get(loop-1).setEndLocation(newStartLocation);
            // }
        }

        // ######
        // If we ARE working on the end effector bone...
        else if (loop === chainLength - 1)
        {
            // Snap the end effector's end location to the target
            // thisBone.setEndLocation(target);
            // let endEffector = chain[loop + 1];
            let endEffector = proxy[loop + 1];
            endEffector.copy(targetPoint);
            // endEffector.matrixWorld.setPosition(targetPoint);

            // Get the UV between the target / end-location (which are now the same) and the start location of this bone
            // Vec3f thisBoneOuterToInnerUV = thisBone.getDirectionUV().negated();
            // endEffector.matrixWorld.decompose(nextBonePosition, nextBoneQuaternion, nextBoneScale);
            nextBonePosition.copy(endEffector);
            thisBoneOuterToInnerUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize().negate();

            // If the end effector is global hinged then we have to snap to it, then keep that
            // resulting outer-to-inner UV in the plane of the hinge rotation axis
            switch (thisBoneJointType)
            {
                case JointType.BALL:
            //         // Ball joints do not get constrained on this forward pass
                    break;
                case JointType.GLOBAL_HINGE:
            //         // Global hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
            //         thisBoneOuterToInnerUV = thisBoneOuterToInnerUV.projectOntoPlane( thisBoneJoint.getHingeRotationAxis() );
                    break;
                case JointType.LOCAL_HINGE:
            //         // Local hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
            //
            //         // Construct a rotation matrix based on the previous bones inner-to-to-inner direction...
            //         Mat3f m = Mat3f.createRotationMatrix( mChain.get(loop-1).getDirectionUV() );
                    let relativeHingeRotationAxis = new Vector3();
                    let previousBone = proxy[loop - 1];
                    let previousBoneInnerToOuterUV = new Vector3();
                    previousBoneInnerToOuterUV.copy(thisBonePosition).addScaledVector(previousBone, -1).normalize();
                    let basisVector = new Vector3(0, 1, 0); // TODO debug
                    let q = new Quaternion();
                    q.setFromUnitVectors(basisVector, previousBoneInnerToOuterUV);
            //
            //         // ...and transform the hinge rotation axis into the previous bones frame of reference.
            //         Vec3f relativeHingeRotationAxis = m.times( thisBoneJoint.getHingeRotationAxis() ).normalise();
                    relativeHingeRotationAxis.copy(limitation).applyQuaternion(q);

            //
            //         // Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
            //         // Note: The returned vector is normalised.
            //         thisBoneOuterToInnerUV = thisBoneOuterToInnerUV.projectOntoPlane(relativeHingeRotationAxis);
                    thisBoneOuterToInnerUV.projectOnPlane(relativeHingeRotationAxis);
                    break;
            }
            //
            // // Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
            // // multiplied by the length of the bone.
            // Vec3f newStartLocation = target.plus( thisBoneOuterToInnerUV.times(thisBoneLength) );
            let newStartLocation = new Vector3();
            newStartLocation.copy(targetPoint).addScaledVector(thisBoneOuterToInnerUV, thisBoneLength);

            //
            // // Set the new start joint location for this bone to be new start location...
            // thisBone.setStartLocation(newStartLocation);
            // thisBone.position.copy(newStartLocation);
            thisBone.copy(newStartLocation);
            //
            // // ...and set the end joint location of the bone further in to also be at the new start location (if there IS a bone
            // // further in - this may be a single bone chain)
            // if (loop > 0)
            // {
            //     mChain.get(loop-1).setEndLocation(newStartLocation);
            // }
        }
    } // End of forward pass
};

// ---------- Backward pass from base to end effector -----------
FABRIK.prototype.backward = function(
    targetPoint, chain, constraints
)
{
    let mBaseboneConstraintType = BaseboneConstraintType3D.NONE;
    const base = constraints.links[0];
    if (base.id !== 0) throw Error('Please specify all links in the constraints array.');
    const limitation = base.limitation;
    const rotationMin = base.rotationMin; // unsupported?
    const rotationMax = base.rotationMax;
    if (limitation)
        mBaseboneConstraintType = BaseboneConstraintType3D.LOCAL_HINGE;
    else if (rotationMin || rotationMax)
        mBaseboneConstraintType = BaseboneConstraintType3D.NONE;
    else
        mBaseboneConstraintType = BaseboneConstraintType3D.NONE;

    const thisBonePosition = this.v1;
    // const thisBoneScale = this.v2;
    // const thisBoneQuaternion= this.q1;
    const previousBonePosition = this.v3;
    // const previousBoneScale = this.v4;
    // const previousBoneQuaternion = this.q2;

    const thisBoneInnerToOuterUV = this.v5;
    const prevBoneInnerToOuterUV = this.v6;

    const nextBonePosition = this.v7;
    // const nextBoneScale = this.v8;
    // const nextBoneQuaternion = this.q3;

    const proxy = constraints.chainProxy;
    const boneLengths = constraints.boneLengths;
    const chainLength = chain.length - 1; // end bone === end effector!

    for (let loop = 0; loop < chainLength; ++loop)
    {
        // FabrikBone3D thisBone = mChain.get(loop);
        // float thisBoneLength  = thisBone.length();
        // let thisBone = chain[loop];
        let thisBone = proxy[loop];
        let thisBoneLength = boneLengths[loop];
        thisBonePosition.copy(thisBone);
        // thisBone.matrixWorld.decompose(thisBonePosition, thisBoneQuaternion, thisBoneScale);

        // If we are not working on the basebone
        if (loop !== 0)
        {
            // let previousBone = chain[loop - 1];
            let previousBone = proxy[loop - 1];
            // previousBone.matrixWorld.decompose(previousBonePosition, previousBoneQuaternion, previousBoneScale);
            previousBonePosition.copy(previousBone);
            // let thisEnd = chain[loop + 1];
            let thisEnd = proxy[loop + 1];
            // thisEnd.matrixWorld.decompose(nextBonePosition, nextBoneQuaternion, nextBoneScale);
            nextBonePosition.copy(thisEnd);
            // Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
            // Vec3f thisBoneInnerToOuterUV = thisBone.getDirectionUV();
            // Vec3f prevBoneInnerToOuterUV = mChain.get(loop-1).getDirectionUV();
            thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
            prevBoneInnerToOuterUV.copy(thisBonePosition).addScaledVector(previousBonePosition, -1).normalize();

            // Dealing with a ball joint?
            // FabrikJoint3D thisBoneJoint = thisBone.getJoint();
            // JointType jointType = thisBoneJoint.getJointType();
            let jointType; // = JointType.BALL;

            const link = constraints.links[loop];
            const id = link.id;
            if (id !== loop) throw Error('Please specify all links in the constraints array.');
            const limitation = link.limitation;
            const rotationMin = link.rotationMin;
            const rotationMax = link.rotationMax;
            if (limitation)
                jointType = JointType.LOCAL_HINGE;
            else if (rotationMin || rotationMax)
                jointType = JointType.BALL;
            else
                jointType = JointType.BALL;

            if (jointType === JointType.BALL)
            {
            //     float angleBetweenDegs    = Vec3f.getAngleBetweenDegs(prevBoneInnerToOuterUV, thisBoneInnerToOuterUV);
            //     float constraintAngleDegs = thisBoneJoint.getBallJointConstraintDegs();

                // Keep this bone direction constrained within the rotor about the previous bone direction
                // if (angleBetweenDegs > constraintAngleDegs)
                // {
                //     thisBoneInnerToOuterUV = Vec3f.getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, prevBoneInnerToOuterUV, constraintAngleDegs);
                // }
            }
            else if (jointType === JointType.GLOBAL_HINGE)
            {
                // Get the hinge rotation axis and project our inner-to-outer UV onto it
                // Vec3f hingeRotationAxis  =  thisBoneJoint.getHingeRotationAxis();
                // thisBoneInnerToOuterUV = thisBoneInnerToOuterUV.projectOntoPlane(hingeRotationAxis);

                // If there are joint constraints, then we must honour them...
                // float cwConstraintDegs   = -thisBoneJoint.getHingeClockwiseConstraintDegs();
                // float acwConstraintDegs  =  thisBoneJoint.getHingeAnticlockwiseConstraintDegs();
                // if ( !( Utils.approximatelyEquals(cwConstraintDegs, -FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.001f) ) &&
                // !( Utils.approximatelyEquals(acwConstraintDegs, FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.001f) ) )
                // {
                //     Vec3f hingeReferenceAxis =  thisBoneJoint.getHingeReferenceAxis();

                    // Get the signed angle (about the hinge rotation axis) between the hinge reference axis and the hinge-rotation aligned bone UV
                    // Note: ACW rotation is positive, CW rotation is negative.
                    // float signedAngleDegs = Vec3f.getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);

                    // Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
                    // if (signedAngleDegs > acwConstraintDegs)
                    // {
                    //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalised();
                    // }
                    // else if (signedAngleDegs < cwConstraintDegs)
                    // {
                    //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalised();
                    // }
                // }
            }
            else if (jointType === JointType.LOCAL_HINGE)
            {
                let relativeHingeRotationAxis = new Vector3();
                let previousBone = proxy[loop - 1];
                let previousBoneInnerToOuterUV = new Vector3();
                previousBoneInnerToOuterUV.copy(thisBonePosition).addScaledVector(previousBone, -1).normalize();
                // Transform the hinge rotation axis to be relative to the previous bone in the chain
                // Vec3f hingeRotationAxis  = thisBoneJoint.getHingeRotationAxis();

                let basisVector = new Vector3(0, 1, 0); // TODO debug
                let q = new Quaternion();
                q.setFromUnitVectors(basisVector, previousBoneInnerToOuterUV);
                // Construct a rotation matrix based on the previous bone's direction
                // Mat3f m = Mat3f.createRotationMatrix(prevBoneInnerToOuterUV);

                relativeHingeRotationAxis.copy(limitation).applyQuaternion(q);
                // Transform the hinge rotation axis into the previous bone's frame of reference
                // Vec3f relativeHingeRotationAxis  = m.times(hingeRotationAxis).normalise();

                thisBoneInnerToOuterUV.projectOnPlane(relativeHingeRotationAxis);
                // Project this bone direction onto the plane described by the hinge rotation axis
                // Note: The returned vector is normalised.
                // thisBoneInnerToOuterUV = thisBoneInnerToOuterUV.projectOntoPlane(relativeHingeRotationAxis);

                // Constrain rotation about reference axis if required
                // float cwConstraintDegs   = -thisBoneJoint.getHingeClockwiseConstraintDegs();
                let cwConstraintDegs   = rotationMin;
                // float acwConstraintDegs  =  thisBoneJoint.getHingeAnticlockwiseConstraintDegs();
                let acwConstraintDegs   = rotationMax;
                // if ( !( Utils.approximatelyEquals(cwConstraintDegs, -FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.001f) ) &&
                // !( Utils.approximatelyEquals(acwConstraintDegs, FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.001f) ) )
                if (cwConstraintDegs !== undefined || acwConstraintDegs !== undefined)
                {
                    // Calc. the reference axis in local space
                    // Vec3f relativeHingeReferenceAxis = m.times( thisBoneJoint.getHingeReferenceAxis() ).normalise();
                    let relativeHingeReferenceAxis = new Vector3();
                    relativeHingeReferenceAxis.copy(basisVector);

                    // Get the signed angle (about the hinge rotation axis) between the hinge reference axis and the hinge-rotation aligned bone UV
                    // Note: ACW rotation is positive, CW rotation is negative.
                    // float signedAngleDegs = Vec3f.getSignedAngleBetweenDegs(relativeHingeReferenceAxis, thisBoneInnerToOuterUV, relativeHingeRotationAxis);
                    let unsignedAngle = Math.acos(relativeHingeReferenceAxis.dot(thisBoneInnerToOuterUV));
                    if (unsignedAngle < -1 || unsignedAngle > 1) console.log('error');
                    let cross = new Vector3();
                    cross.crossVectors(relativeHingeReferenceAxis, thisBoneInnerToOuterUV);
                    let sign = Math.sign(cross.dot(relativeHingeRotationAxis));
                    let signedAngleDegs = sign * unsignedAngle;

                    // Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
                    if (acwConstraintDegs !== undefined && signedAngleDegs > acwConstraintDegs)
                    {
                    //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(relativeHingeReferenceAxis, acwConstraintDegs, relativeHingeRotationAxis).normalise();
                    }
                    else if (cwConstraintDegs !== undefined && signedAngleDegs < cwConstraintDegs)
                    {
                    //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(relativeHingeReferenceAxis, cwConstraintDegs, relativeHingeRotationAxis).normalise();
                    }
                }

            } // End of local hinge section

            // At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
            // so we can set the new inner joint location to be the end joint location of this bone plus the
            // outer-to-inner direction unit vector multiplied by the length of the bone.
            // Vec3f newEndLocation = thisBone.getStartLocation().plus( thisBoneInnerToOuterUV.times(thisBoneLength) );
            const newEndLocation = new Vector3();
            newEndLocation.copy(thisBonePosition).addScaledVector(thisBoneInnerToOuterUV, thisBoneLength);

            // Set the new start joint location for this bone
            // thisBone.setEndLocation(newEndLocation);
            // thisEnd.position.copy(newEndLocation);
            thisEnd.copy(newEndLocation);

            // If we are not working on the end effector bone, then we set the start joint location of the next bone in
            // the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
            // if (loop < chainLength - 1) {
            //     mChain.get(loop+1).setStartLocation(newEndLocation);
            // }
        }

        // ######
        // If we ARE working on the basebone...
        else
        {
            let thisEnd = proxy[loop + 1];
            nextBonePosition.copy(thisEnd);

            // If the base location is fixed then snap the start location of the basebone back to the fixed base...
            if (mFixedBaseMode)
            {
                // thisBone.setStartLocation(mFixedBaseLocation);
                // thisBone.position.copy(this.mFixedBaseLocation);
                thisBone.copy(this.mFixedBaseLocation);
            }
            else // ...otherwise project it backwards from the end to the start by its length.
            {
                // thisBone.setStartLocation( thisBone.getEndLocation().minus( thisBone.getDirectionUV().times(thisBoneLength) ) );
                // let thisEnd = chain[loop + 1];
                // thisEnd.matrixWorld.decompose(nextBonePosition, nextBoneQuaternion, nextBoneScale);
                thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
                // thisBone.position.copy(nextBonePosition).addScaledVector(thisBoneInnerToOuterUV, -thisBoneLength);
                thisBone.copy(nextBonePosition).addScaledVector(thisBoneInnerToOuterUV, -thisBoneLength);
            }

            thisBonePosition.copy(thisBone);

            // If the basebone is unconstrained then process it as usual...
            if (mBaseboneConstraintType === BaseboneConstraintType3D.NONE)
            {
                // Set the new end location of this bone, and if there are more bones,
                // then set the start location of the next bone to be the end location of this bone
                // Vec3f newEndLocation = thisBone.getStartLocation().plus( thisBone.getDirectionUV().times(thisBoneLength) );
                thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
                let newEndLocation = new Vector3();
                newEndLocation.copy(thisBone).addScaledVector(thisBoneInnerToOuterUV, thisBoneLength);
                thisEnd.copy(newEndLocation);

                // thisBone.setEndLocation(newEndLocation);

                // if (mChain.size() > 1) {
                //     mChain.get(1).setStartLocation(newEndLocation);
                // }
            }
            else // ...otherwise we must constrain it to the basebone constraint unit vector
            {
                if (mBaseboneConstraintType === BaseboneConstraintType3D.GLOBAL_ROTOR)
                {
                    // Get the inner-to-outer direction of this bone
                    // Vec3f thisBoneInnerToOuterUV = thisBone.getDirectionUV();

                    // float angleBetweenDegs    = Vec3f.getAngleBetweenDegs(mBaseboneConstraintUV, thisBoneInnerToOuterUV);
                    // float constraintAngleDegs = thisBone.getBallJointConstraintDegs();

                    // if (angleBetweenDegs > constraintAngleDegs)
                    // {
                    //     thisBoneInnerToOuterUV = Vec3f.getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, mBaseboneConstraintUV, constraintAngleDegs);
                    // }

                    // Vec3f newEndLocation = thisBone.getStartLocation().plus( thisBoneInnerToOuterUV.times(thisBoneLength) );

                    // thisBone.setEndLocation( newEndLocation );

                    // Also, set the start location of the next bone to be the end location of this bone
                    // if (mChain.size() > 1) {
                    //     mChain.get(1).setStartLocation(newEndLocation);
                    // }
                }
                else if (mBaseboneConstraintType === BaseboneConstraintType3D.LOCAL_ROTOR)
                {
                    // Note: The mBaseboneRelativeConstraintUV is updated in the FabrikStructure3D.solveForTarget()
                    // method BEFORE this FabrikChain3D.solveForTarget() method is called. We no knowledge of the
                    // direction of the bone we're connected to in another chain and so cannot calculate this
                    // relative basebone constraint direction on our own, but the FabrikStructure3D does it for
                    // us so we are now free to use it here.

                    // Get the inner-to-outer direction of this bone
                    // Vec3f thisBoneInnerToOuterUV = thisBone.getDirectionUV();

                    // Constrain about the relative basebone constraint unit vector as neccessary
                    // float angleBetweenDegs    = Vec3f.getAngleBetweenDegs(mBaseboneRelativeConstraintUV, thisBoneInnerToOuterUV);
                    // float constraintAngleDegs = thisBone.getBallJointConstraintDegs();
                    // if (angleBetweenDegs > constraintAngleDegs)
                    // {
                    //     thisBoneInnerToOuterUV = Vec3f.getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, mBaseboneRelativeConstraintUV, constraintAngleDegs);
                    // }

                    // Set the end location
                    // Vec3f newEndLocation = thisBone.getStartLocation().plus( thisBoneInnerToOuterUV.times(thisBoneLength) );
                    // thisBone.setEndLocation( newEndLocation );

                    // Also, set the start location of the next bone to be the end location of this bone
                    // if (mChain.size() > 1) {
                    //     mChain.get(1).setStartLocation(newEndLocation);
                    // }
                }
                else if (mBaseboneConstraintType === BaseboneConstraintType3D.GLOBAL_HINGE)
                {
                //     FabrikJoint3D thisJoint  =  thisBone.getJoint();
                //     Vec3f hingeRotationAxis  =  thisJoint.getHingeRotationAxis();
                //     float cwConstraintDegs   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
                //     float acwConstraintDegs  =  thisJoint.getHingeAnticlockwiseConstraintDegs();

                    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
                    // Vec3f thisBoneInnerToOuterUV = thisBone.getDirectionUV().projectOntoPlane(hingeRotationAxis);

                    // If we have a global hinge which is not freely rotating then we must constrain about the reference axis
                    // if ( !( Utils.approximatelyEquals(cwConstraintDegs , -FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.01f) &&
                    // Utils.approximatelyEquals(acwConstraintDegs,  FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.01f) ) )
                    // {
                        // Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
                        // rotation axis). Note: ACW rotation is positive, CW rotation is negative.
                        // Vec3f hingeReferenceAxis = thisJoint.getHingeReferenceAxis();
                        // float signedAngleDegs    = Vec3f.getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);

                        // Constrain as necessary
                        // if (signedAngleDegs > acwConstraintDegs)
                        // {
                        //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalise();
                        // }
                        // else if (signedAngleDegs < cwConstraintDegs)
                        // {
                        //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalise();
                        // }
                    // }

                    // Calc and set the end location of this bone
                    // Vec3f newEndLocation = thisBone.getStartLocation().plus( thisBoneInnerToOuterUV.times(thisBoneLength) );
                    // thisBone.setEndLocation( newEndLocation );

                    // Also, set the start location of the next bone to be the end location of this bone
                    // if (mChain.size() > 1) {
                    //     mChain.get(1).setStartLocation(newEndLocation);
                    // }
                }
                else if (mBaseboneConstraintType === BaseboneConstraintType3D.LOCAL_HINGE)
                {
                    // FabrikJoint3D thisJoint  =  thisBone.getJoint();
                    // Vec3f hingeRotationAxis  =  mBaseboneRelativeConstraintUV;                   // Basebone relative constraint is our hinge rotation axis!
                    let hingeRotationAxis = limitation;
                    // float cwConstraintDegs   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
                    // float acwConstraintDegs  =  thisJoint.getHingeAnticlockwiseConstraintDegs();

                    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
                    // Vec3f thisBoneInnerToOuterUV = thisBone.getDirectionUV().projectOntoPlane(hingeRotationAxis);
                    thisBoneInnerToOuterUV.copy(nextBonePosition).addScaledVector(thisBonePosition, -1).normalize();
                    thisBoneInnerToOuterUV.projectOnPlane(hingeRotationAxis);

                    // If we have a local hinge which is not freely rotating then we must constrain about the reference axis
                    // if ( !( Utils.approximatelyEquals(cwConstraintDegs , -FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.01f) &&
                    // Utils.approximatelyEquals(acwConstraintDegs,  FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS, 0.01f) ) )
                    // {
                        // Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
                        // rotation axis). Note: ACW rotation is positive, CW rotation is negative.
                        // Vec3f hingeReferenceAxis = mBaseboneRelativeReferenceConstraintUV;
                        // float signedAngleDegs    = Vec3f.getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);

                        // Constrain as necessary
                        // if (signedAngleDegs > acwConstraintDegs)
                        // {
                        //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalise();
                        // }
                        // else if (signedAngleDegs < cwConstraintDegs)
                        // {
                        //     thisBoneInnerToOuterUV = Vec3f.rotateAboutAxisDegs(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalise();
                        // }
                    // }

                    // Calc and set the end location of this bone
                    // Vec3f newEndLocation = thisBone.getStartLocation().plus( thisBoneInnerToOuterUV.times(thisBoneLength) );
                    // thisBone.setEndLocation( newEndLocation );
                    let newEndLocation = new Vector3();
                    newEndLocation.copy(thisBone).addScaledVector(thisBoneInnerToOuterUV, thisBoneLength);
                    thisEnd.copy(newEndLocation);

                    // Also, set the start location of the next bone to be the end location of this bone
                    // [madblade] Unnecessary
                    // if (mChain.size() > 1) {
                    //     mChain.get(1).setStartLocation(newEndLocation);
                    // }
                }

            } // End of basebone constraint handling section

        } // End of basebone handling section

    } // End of backward-pass loop over all bones
};

export { FABRIK };
