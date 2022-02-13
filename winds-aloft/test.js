/* global describe, it */

const assert = require('assert')
const winds = require('./index')

describe('winds-aloft', () => {
  it('should exist', () => {
    assert(winds !== undefined)
  })
  it('should expose an FD1 function', () => {
    assert(winds.FD1 !== undefined)
  })
})
