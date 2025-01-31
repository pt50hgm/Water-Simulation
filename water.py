import sys
import math
import random
import pygame
from pygame.locals import QUIT

FPS = 60
SCREEN_W = 600
SCREEN_H = 500

rows = 20
columns = 40
particleAmount = rows*columns
spawnRect = [150, 250, 300, 500]

gravity = 20
collisionDamping = 0.6
smoothingR = 14
mass = 1
targetDensity = 5.2
pressureMult = 27

# rows = 42
# columns = 42
# particleAmount = rows*columns
# spawnRect = [250, 300, 500, 600]

# gravity = 0
# collisionDamping = 0.95
# smoothingR = 14
# mass = 1
# targetDensity = 5.2
# pressureMult = 27

pygame.init()
DISPLAYSURF = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption('Water Simulation')

clock = pygame.time.Clock()
dt = 0
mouseIsPressed = False

partitions = particleAmount * 10000
spatialLookup = [None for i in range(particleAmount)]
startIndices = [None for i in range(partitions)]
nearbyParticles = [None for i in range(partitions)]

def GetRandomDir():
  rAngle = random.uniform(0, 360)
  return (pygame.math.Vector2(math.cos(math.radians(rAngle)), math.sin(math.radians(rAngle))))

def SmoothingKernel(r, dist):
  if dist >= r:
    return 0
  volume = math.pi * r**4 / 6
  return (r - dist)**2 / volume
  
def SmoothingKernelDerivative(r, dist):
  if dist >= r:
    return 0
  scale = 12 / (math.pi * r**4)
  return (r - dist) * scale

def CalculateDensity(sample, particlesList):
  density = 0
  for particle in particlesList:
    dist = (particle.predictedPos - sample).magnitude()
    influence = SmoothingKernel(smoothingR, dist)
    density += mass * influence
  return density

def ConvertDensityToPressure(density):
  densityError = density - targetDensity
  pressure = densityError * pressureMult
  return pressure

def UpdateSpatialLookup():
  for i in range(len(particles)):
    cellKey = CellToHash(PositionToCellCoord(particles[i].pos))
    spatialLookup[i] = (cellKey, i)
    startIndices[i] = None
    nearbyParticles[i] = None

  spatialLookup.sort(key=lambda x: x[0])
  
  for i in range(len(particles)):
    key = spatialLookup[i][0]
    if i == 0:
      startIndices[key] = i
    elif key != spatialLookup[i-1][0]:
      startIndices[key] = i

def PositionToCellCoord(pos):
  cellX = math.floor(pos.x/smoothingR)
  cellY = math.floor(pos.y/smoothingR)
  return (cellX, cellY)

def CellToHash(coord):
  return (coord[0]*15823 + coord[1]*9737333) % partitions

def FindEachPointWithinRadius(sample):
  global nearbyParticles

  points = []
  centreX, centreY = PositionToCellCoord(sample)
  sqrR = smoothingR**2
  nearbyList = nearbyParticles[CellToHash((centreX, centreY))]
  if nearbyList != None:
    return nearbyList
  
  for offsetX, offsetY in ((-1,-1), (-1,0), (-1,1), (0,-1), (0,0), (0,1), (1,-1), (1,0), (1,1)):
    key = CellToHash((centreX + offsetX, centreY + offsetY))
    cellStartIndex = startIndices[key]
    if cellStartIndex == None:
      continue
    for i in range(cellStartIndex, len(spatialLookup)):
      if spatialLookup[i][0] != key:
        break

      particleIndex = spatialLookup[i][1]
      sqrDist = (particles[particleIndex].pos - sample).magnitude()

      if sqrDist <= sqrR:
        points.append(particles[particleIndex])
  nearbyList = points
  return points

def SimulationStep():
  UpdateSpatialLookup()
  
  for particle in particles:
    particle.vel.y += gravity * dt
    particle.predictedPos = particle.pos + particle.vel * 1/60

  for particle in particles:
    particle.nearbyParticlesList = FindEachPointWithinRadius(particle.predictedPos)
  
  for particle in particles:
    particle.density = CalculateDensity(particle.predictedPos, particle.nearbyParticlesList)
    

  for particle in particles:
    pressureForce = particle.CalculatePressureForce(particle.nearbyParticlesList)
    pressureAccel = pressureForce / particle.density
    particle.vel += pressureAccel * dt
    particle.vel += particle.interactionForce
    if particle.vel.magnitude() > 100:
      particle.vel = particle.vel.normalize() * 100
  
  for particle in particles:
    particle.pos += particle.vel * dt
    particle.ResolveCollisions()





class Particle:
  def __init__(self, pos, vel):
    self.pos = pos
    self.vel = vel
    self.r = 2
    self.density = None
    self.property = 1
    self.predictedPos = None
    self.interactionForce = pygame.math.Vector2()
  
  def Update(self):
    colour = pygame.Color(0, 0, 220)
    colour.hsva = (250 - min(round(self.vel.magnitude() / 100 * 250), 250), 100, 50 + min(50, round(self.vel.magnitude())))
    pygame.draw.circle(DISPLAYSURF, colour, (self.pos.x, self.pos.y), self.r)
  
  def ResolveCollisions(self):
    if self.pos.x > SCREEN_W-self.r:
      self.pos.x = SCREEN_W-self.r
      self.vel.x *= -collisionDamping
    elif self.pos.x < 0+self.r:
      self.pos.x = 0+self.r
      self.vel.x *= -collisionDamping
    if self.pos.y > SCREEN_H-self.r:
      self.pos.y = SCREEN_H-self.r
      self.vel.y *= -collisionDamping
    elif self.pos.y < 0+self.r:
      self.pos.y = 0+self.r
      self.vel.y *= -collisionDamping

  def CalculatePressureForce(self, particlesList):
    pressureForce = pygame.math.Vector2()
    for particle in particlesList:
      if particle == self:
        continue
      dist = (particle.predictedPos - self.predictedPos).magnitude()
      if particle.predictedPos == self.predictedPos:
        dir = GetRandomDir()
      else:
        dir = (particle.predictedPos - self.predictedPos).normalize()
      slope = SmoothingKernelDerivative(smoothingR, dist)
      density = particle.density
      sharedPressure = (ConvertDensityToPressure(density) + ConvertDensityToPressure(self.density))/2
      pressureForce += sharedPressure * dir * slope * mass / density
    return pressureForce
  
  def CalculateInteractionForce(self, r, strength, mousePos):
    self.interactionForce = pygame.math.Vector2()
    offset = mousePos - self.pos
    sqrDist = pygame.math.Vector2.dot(offset, offset)
    if sqrDist < r*r:
      dist = math.sqrt(sqrDist)
      if dist == 0:
        dir = GetRandomDir()
      else:
        dir = offset.normalize()
      centreT = 1 - dist/r
      self.interactionForce += (dir * strength) * centreT
    


def CreateParticles(spawnRect, rows, columns):
  x, y, w, h = spawnRect
  for i in range(columns):
    for j in range(rows):
        particles.append(Particle(pygame.math.Vector2(x + (i-columns/2 + 0.5) * w/columns + random.uniform(-2,2), y + (j-rows/2 + 0.5) * h/rows + random.uniform(-2,2)), pygame.math.Vector2()))

particles = []

CreateParticles(spawnRect, rows, columns)

while True:
  dt = clock.tick(FPS)/1000
  pygame.draw.rect(DISPLAYSURF, (0, 0, 0), (0, 0, SCREEN_W, SCREEN_H))
  
  for event in pygame.event.get():
    if event.type == QUIT:
      pygame.quit()
      sys.exit()
    elif event.type == pygame.MOUSEBUTTONDOWN:
      mouseIsPressed = True
    elif event.type == pygame.MOUSEBUTTONUP:
      mouseIsPressed = False
  mouseX, mouseY = pygame.mouse.get_pos()
  
  for particle in particles:
    particle.interactionForce = pygame.math.Vector2()
  if mouseIsPressed:
    for particle in particles:
      particle.CalculateInteractionForce(70, -160, pygame.math.Vector2(mouseX, mouseY))

  SimulationStep()
  
  for particle in particles:
    particle.Update()
  
  pygame.display.flip()