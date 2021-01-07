
// from https://github.com/jsantell/THREE.IK/blob/master/src/IKChain.js
// https://github.com/jsantell/THREE.IK/blob/master/src/IK.js

import {
    Quaternion
} from 'three';

function FABRIK()
{
    this.q = new Quaternion();
}

FABRIK.prototype.solve = function(
    chain,
    targetPoint,
    iterations,
    constraints
)
{

}

FABRIK.prototype.forward = function()
{

}

FABRIK.prototype.backward = function()
{

}

export { FABRIK };
