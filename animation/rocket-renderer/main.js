import * as THREE from "./node_modules/three/build/three.module.js"
import { OrbitControls } from './node_modules/three/examples/jsm/controls/OrbitControls.js'
import * as dat from 'dat.gui'
console.log(dat)
import { NormalAnimationBlendMode } from "three"

/*
Idea for final product:
User inputs lat and long. Let this point be point P.
Fetches topography and satellite data x kilometers from P in from somewhere
Renders rocket at P
Reads through csv, fetches lat, long, rotation, and altitude data
Moves rocket as such
*/

//create scene, camera, renderer

const scene = new THREE.Scene()

const camera = new THREE.PerspectiveCamera(75, innerWidth/innerHeight, 0.1, 1000)
camera.position.z = 50
camera.position.set( 0, 0, 20 );

const renderer = new THREE.WebGLRenderer()
renderer.setSize(innerWidth, innerHeight)
renderer.setPixelRatio(devicePixelRatio)
document.body.appendChild(renderer.domElement)

//allows for drag to move around thing
const controls = new OrbitControls( camera, renderer.domElement );
controls.addEventListener('change',render);


//rerenders the scene
function render(){
  renderer.render(scene, camera);
}


//TODO: make this automatic
const heightmaps = {
  '1m_normal': './heightmaps/1m/heightmap_1m_normal.png',
  '1m_auto': './heightmaps/1m/heightmap_1m_auto.png',
  '10m_normal': './heightmaps/10m/heightmap_10m_normal.png',
  '10m_auto': './heightmaps/10m/heightmap_10m_auto.png',
  'b1_normal': './heightmaps/b1/heightmap_b1_normal.png',
  'b1_auto': './heightmaps/b1/heightmap_b1_auto.png',
  'b1_auto_smooth': './heightmaps/b1/heightmap_b1_auto_smoothed.png',
  'b3ql1_normal': './heightmaps/b3ql1/heightmap_b3ql1_normal.png',
  'b3ql1_normal+corrected': './heightmaps/b3ql1/heightmap_b3ql1_normal_corrected.png',
  'b3ql1_normal+corrected2': './heightmaps/b3ql1/heightmap_b3ql1_normal_corrected2.png',
  'b3ql1_auto': './heightmaps/b3ql1/heightmap_b3ql1_auto.png',

  'tangram': '/heightmaps/tangram/heightmap_tangram.png'
}


//dictionary to hold all the variables in
const world = {
  plane: {
    color: "#FFFFFF",
    heightmap: heightmaps[ 'b1_auto'],
    disScale: 7,
    width: 46.8390805,
    height: 100,
    widthSegments: 1000,
    heightSegments: 1000
  },
  light: {
    color: "#ffcb73",
    castShadow: false,
    intensity: 1,
    x: 40,
    y: -9,
    z: 15
  }
}

//init gui
const gui = new dat.GUI()

gui.add(world.plane, "heightmap", heightmaps).onChange(generatePlane)
gui.add(world.plane, 'disScale', 1, 250).onChange(generatePlane)
gui.add(world.plane, 'width', 1, 500).onChange(generateGeo)
gui.add(world.plane, "height", 1, 500).onChange(generateGeo)
gui.add(world.plane, "widthSegments", 500, 10000).onChange(generateGeo)
gui.add(world.plane, "heightSegments", 500, 10000).onChange(generateGeo)

//doesn't work
gui.addColor(world.light, 'color').onChange(generateLight)

//doesn't work
gui.add(world.light, 'castShadow').onChange(generateLight)

//doesn't work
gui.add(world.light, "intensity", 1, 10).onChange(generateLight)


gui.add(world.light, 'x', -40, 40).onChange(generateLight)
gui.add(world.light, 'y', -40, 40).onChange(generateLight)
gui.add(world.light, 'z', -40, 40).onChange(generateLight)


gui.addColor(world.plane, 'color').onChange(generateMat)


//TODO: make this not manual. also figure out maybe an api to pull satillite data from
//gui.add(heightmap.choice, "filename")
//const texture = new THREE.TextureLoader().load('./texturemaps/texture_b1_auto.png')
const texture = new THREE.TextureLoader().load('./texturemaps/texture_b1ql3_auto.png')
//const texture = null

function generateMat() {
  planeMesh.material = new THREE.MeshStandardMaterial({
    color: world.plane.color,
    wireframe: false,
    flatShading: true,
    roughness: 1,
    side: THREE.DoubleSide,
    map: texture,
    shadowSide: THREE.DoubleSide,
    displacementMap: new THREE.TextureLoader().load(world.plane.heightmap),
    displacementScale: world.plane.disScale
  })
  renderer.render(scene, camera)
}

function generateGeo() {
  planeMesh.geometry.dispose()
  planeMesh.geometry = new THREE.PlaneGeometry(
    world.plane.width,
    world.plane.height,
    world.plane.widthSegments,
    world.plane.heightSegments
  )
  renderer.render(scene, camera)
}

//this function just doesn't work correctly
//trying to change the intensity and the castShadow value just makes the light disappear
function generateLight(){
  //light = new THREE.DirectionalLight(world.light.color, world.light.intensity)
  // light.color = world.light.color
  // light.intensity = world.light.intensity
  // light.castShadow = world.light.castShadow
  light.position.set(world.light.x, world.light.y, world.light.z)
  renderer.render(scene, camera)
  light.target.updateMatrixWorld();
  console.log(light.color)
}


function generatePlane() {
  planeMesh.geometry.dispose()
  planeMesh.geometry = new THREE.PlaneGeometry(
    world.plane.width,
    world.plane.height,
    world.plane.widthSegments,
    world.plane.heightSegments
  )
  planeMesh.material = new THREE.MeshStandardMaterial({
    color: world.plane.color,
    wireframe: false,
    flatShading: true,
    roughness: 1,
    side: THREE.DoubleSide,
    map: texture,
    shadowSide: THREE.DoubleSide,
    displacementMap: new THREE.TextureLoader().load(world.plane.heightmap),
    displacementScale: world.plane.disScale
  })
  console.log(world.plane.disScale)
  renderer.render(scene, camera)
}






//creates plane's geometry and material
const planeGeometry = new THREE.PlaneGeometry(world.plane.width, world.plane.height, world.plane.widthSegments, world.plane.heightSegments)
const planeMaterial = new THREE.MeshStandardMaterial({
  color: 0x4FB820,
  wireframe: false,
  flatShading: true,
  side: THREE.DoubleSide,
  shadowSide: THREE.DoubleSide,
  map: texture,
  displacementMap: new THREE.TextureLoader().load(world.plane.heightmap),
  displacementScale: world.plane.disScale
})


//creates the plane's mesh
const planeMesh = new THREE.Mesh(planeGeometry, planeMaterial)
scene.add(planeMesh)
generatePlane()


const light = new THREE.DirectionalLight(0xFFFFFF, 2)
light.position.set(40, -9, 15)
scene.add(light)
light.castShadow = true

//this renders the little square with line thing to tell you where the light is
const helper = new THREE.DirectionalLightHelper( light, 1 );
scene.add( helper )
scene.add(light.target)


renderer.render(scene, camera)