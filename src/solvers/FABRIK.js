
// from https://github.com/jsantell/THREE.IK/blob/master/src/IKChain.js
// https://github.com/jsantell/THREE.IK/blob/master/src/IK.js
// https://github.com/jsantell/THREE.IK/issues/2
// https://github.com/jsantell/THREE.IK/issues/3
// https://github.com/jsantell/THREE.IK/issues/4
// http://number-none.com/product/IK%20with%20Quaternion%20Joint%20Limits/index.html
//

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
