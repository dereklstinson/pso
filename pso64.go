/*

pso is based on the slides from Jaco F. Schutte EGM 6365 - Structural Optimization Fall 2005
Link to slides https://www.mii.lt/zilinskas/uploads/Heuristic%20Algorithms/Lectures/Lect4/PSO2.pdf

*/

package pso

import (
	"errors"
	"math"
	"math/rand"
	"sort"
	"sync"
	"time"
)

//Swarm64 contains the particles and meta values
type Swarm64 struct {
	k                                                                                 int
	max                                                                               bool
	fitness                                                                           float64
	cognative, social, vmax, constriction, alphamax, xminstart, xmaxstart, inertiamax float64
	particles                                                                         []particle64
	globalposition                                                                    []float64
	mode                                                                              mode
	source                                                                            rand.Source
	rng                                                                               *rand.Rand
	mux                                                                               *sync.RWMutex
}

//CreateSwarm64 creates a particle swarm. If seed is negative it will initialize the rng source with computer clock.
//If it is positive it will initialize the swarm using the seed passed.
//
//Each particle inside of the swarm will get its own rng based on that seed.
//Passing an non negative int64 to each particle for its own rng.
//There is a posiblilty of having duplicate particles.
//Since the max value of int64 is 9,223,372,036,854,775,807 it is highly improbable.
func CreateSwarm64(seed int) *Swarm64 {
	source := rand.NewSource(int64(time.Now().Nanosecond()))

	return &Swarm64{
		source: source,
		rng:    rand.New(source),
		k:      1,
		mux:    new(sync.RWMutex),
	}
}

//SetVanilla sets the pso to vanilla mode
func (s *Swarm64) SetVanilla(numofparticles int,
	dims int,
	cognative float64,
	social float64,
	vmax float64,
	minpositionstart float64,
	maxpositionstart float64) {
	s.setswarm(s.mode.vanilla(), numofparticles, dims, cognative, social, vmax, minpositionstart, maxpositionstart, 0, 0)
}

//SetConstantInertia sets the particles update local positions based on Constant Inertia algorithm
func (s *Swarm64) SetConstantInertia(numofparticles int,
	dims int,
	cognative float64,
	social float64,
	vmax float64,
	minpositionstart float64,
	maxpositionstart float64,
	inertiamax float64) {
	s.setswarm(s.mode.constantinertia(), numofparticles, dims, cognative, social, vmax, minpositionstart, maxpositionstart, 0, inertiamax)
}

//SetConstriction sets the particles update local positions based on Constriction algorithm
func (s *Swarm64) SetConstriction(
	numofparticles int,
	dims int,
	cognative float64,
	social float64,
	vmax float64,
	minpositionstart float64,
	maxpositionstart float64,
) {
	s.setswarm(s.mode.constriction(), numofparticles, dims, cognative, social, vmax, minpositionstart, maxpositionstart, 0, 0)
}

//SetDynamicInertiaMaxVelocityReduction sets the particals to Dynamic Inertria Max Velocity Reduction
func (s *Swarm64) SetDynamicInertiaMaxVelocityReduction(
	numofparticles, dims int,
	cognative float64,
	social float64,
	vmaxgamma float64,
	minpositionstart float64,
	maxpositionstart float64,
	inertiamax float64) {
	s.setswarm(s.mode.dynamicInertiaMaxVelReduction(), numofparticles, dims, cognative, social, vmaxgamma, minpositionstart, maxpositionstart, 0, inertiamax)
}

//SetLinearInertiaReduce sets the particles to LinearInertiaReduce
func (s *Swarm64) SetLinearInertiaReduce(
	numofparticles int,
	dims int,
	cognative float64,
	social float64,
	vmax float64,
	minpositionstart float64,
	maxpositionstart float64,
	alphamax float64,
	inertiamax float64) {
	s.setswarm(s.mode.inertiareduction(), numofparticles, dims, cognative, social, vmax, minpositionstart, maxpositionstart, alphamax, inertiamax)

}

//SetFitness sets the PSO's to either fitness.
//
//If max is true. It will try to maximize.
//
//If max is false. It will try to minimize.
//
//Default is false.
//
//This can be switched at any time.
func (s *Swarm64) SetFitness(max bool) {

	s.max = max
	if s.k < 2 {
		if max {
			s.fitness = -99999999
		} else {
			s.fitness = 99999999
		}
		if s.particles != nil {
			for i := range s.particles {
				s.particles[i].fitness = s.fitness
			}
		}
	}
}

//CreateSwarm64 creates a particle swarm
func (s *Swarm64) setswarm(mode mode,
	numofparticles int,
	dims int,
	cognative float64,
	social float64,
	vmax float64,
	pminstart float64,
	pmaxstart float64,
	alphamax float64,
	inertiamax float64) {
	s.globalposition = make([]float64, dims)
	s.cognative = cognative
	s.social = social
	s.vmax = vmax
	s.xminstart = pminstart
	s.xmaxstart = pmaxstart
	s.particles = make([]particle64, numofparticles)
	s.inertiamax = inertiamax
	s.alphamax = alphamax
	gamma := float64(social + cognative)
	s.constriction = float64(2 / (2 - gamma - math.Sqrt((gamma*gamma)-4*gamma)))

	if s.max {
		s.fitness = -99999999
	} else {
		s.fitness = 99999999
	}
	for i := range s.particles {
		s.particles[i] = createparticle64(vmax, pminstart, pmaxstart, alphamax, inertiamax, dims, s.rng.Int63(), s.max)

	}

}

//ResetParticles resets the particles based on the index array passed
func (s *Swarm64) ResetParticles(indexes []int, resetglobalposition bool) error {
	numofparticles := len(s.particles)
	if len(indexes) > numofparticles {
		return errors.New("Length of indexes larger than particle number")
	}
	if resetglobalposition {
		for i := range s.globalposition {
			s.globalposition[i] = 0
		}
	}

	for i := range indexes {
		s.particles[indexes[i]].reset(s.vmax, s.xminstart, s.xmaxstart, s.alphamax, s.inertiamax)
	}

	return nil
}

//AsyncUpdate does the update asyncrounusly
func (s *Swarm64) AsyncUpdate(index int, fitness float64) error {
	if index >= len(s.particles) {
		return errors.New("Index Out Of Bounds")
	}
	switch s.max {
	case true:
		if fitness > s.fitness {
			s.fitness = fitness
			copy(s.globalposition, s.particles[index].position)
		}
	default:
		if fitness < s.fitness {
			s.fitness = fitness
			copy(s.globalposition, s.particles[index].position)
		}
	}
	s.particles[index].isbest(fitness, s.max)
	s.particles[index].update(s.mode, s.cognative, s.social, s.vmax, s.constriction, s.globalposition)
	s.k++
	return nil
}

//GlobalFitness returns how fit the swarm is.
func (s *Swarm64) GlobalFitness() float64 {
	return s.fitness
}

//GlobalPosition returns the swarm best global position.
func (s *Swarm64) GlobalPosition() []float64 {
	return s.globalposition
}

//ParticlePosition returns the particle position of the index passed
func (s *Swarm64) ParticlePosition(index int) []float64 {

	return s.particles[index].position

}

//ParticleFitness returns the fitness of particle at indexed location
func (s *Swarm64) ParticleFitness(index int) float64 {
	return s.particles[index].fitness
}

//AllFitnesses will fill the previousfitnesses slice with values and then return it.
//I did it this way so that if user of this package doesn't want to keep on allocating
//memory then they can pass a already allocated slice.
//
//Here are the rules:
//
//	if previousfitnesses==nil || len(previousfitnesses)!=len(hidden particles) then
//	method will allocate new memory and return the fitnesses of the current particles.
func (s *Swarm64) AllFitnesses(previousfitnesses []float64) []float64 {
	if previousfitnesses == nil || len(previousfitnesses) != len(s.particles) {
		previousfitnesses = make([]float64, len(s.particles))
	}
	for i := range previousfitnesses {
		previousfitnesses[i] = s.particles[i].fitness
	}
	return previousfitnesses
}

//SyncUpdate updates the particle swarm after all particles tested
func (s *Swarm64) SyncUpdate(fitnesses []float64) error {
	if len(fitnesses) != len(s.particles) {
		return errors.New("Sizes of losses and num of particles not the same")
	}
	position := -1
	for i := range fitnesses {
		s.particles[i].isbest(fitnesses[i], s.max)
		if fitnesses[i] < s.fitness {
			s.fitness = fitnesses[i]
			position = i

		}

	}
	if position > -1 {
		copy(s.globalposition, s.particles[position].position)
	}
	for i := range s.particles {
		s.particles[i].update(s.mode, s.cognative, s.social, s.vmax, s.constriction, s.globalposition)
	}
	s.k++
	return nil
}

//KillParticles kills the partilces in the indexes slice.
func (s *Swarm64) KillParticles(indexes []int) error {
	numofparticles := len(s.particles)
	if len(indexes) > numofparticles {
		return errors.New("Length of indexes larger than particle number")
	}
	index := 0
	npindex := 0
	sort.Ints(indexes)
	reshapedparticles := make([]particle64, len(s.particles)-len(indexes))
	for i := range s.particles {
		for j := index; j < len(indexes); j++ {
			if indexes[j] != i {
				reshapedparticles[npindex] = s.particles[i]
				npindex++
				index++
				break
			}

		}
		if index >= len(indexes) {
			reshapedparticles[npindex] = s.particles[i]
			npindex++

		}

	}
	s.particles = reshapedparticles
	return nil
}

//AddParticles addes particles to swarm from previously set conditions
func (s *Swarm64) AddParticles(num int) {
	newparts := make([]particle64, num)
	for i := range newparts {
		newparts[i] = createparticle64(s.vmax, s.xminstart, s.xmaxstart, s.alphamax, s.inertiamax, len(s.globalposition), s.rng.Int63(), s.max)

	}
	s.particles = append(s.particles, newparts...)
}

//IndvSyncUpdatePart1 of 3out of three allows user to parallelize the syncronous update doing it in parts.
//
//This finds the local best for each particle.
//There might some memory copying in this. Unless the dims are absolutly huge, or you put several of these into
//one worker. It might be faster to not parallelize this part.
func (s *Swarm64) IndvSyncUpdatePart1(particleindex int, fitness float64) {
	s.particles[particleindex].isbest(fitness, s.max)
}

//IndvSyncUpdatePart2 of 3 allows user to parallelize the syncronous update doing it in parts.
//
//Since this sets the global best fitness and maybe sets the global best position. This part
//isn't parallelized
func (s *Swarm64) IndvSyncUpdatePart2(fitnesses []float64) {

	position := -1
	for i := range fitnesses {

		if fitnesses[i] < s.fitness {
			s.fitness = fitnesses[i]
			position = i

		}

	}
	if position > -1 {
		copy(s.globalposition, s.particles[position].position)
	}
	s.k++
}

//IndvSyncUpdatePart3 of 3 allows user to parallelize the syncronous update doing it in parts.
//
//Updates The Particles - Most computative part of all all 3. Most to gain from parallelism
//
//If globalposition is nil or not the same size as the hidden global position.  New memory will be allocated.
//For increased speed have this preallocated. Each parallel process should get its own copy of global position.
//GetGlobalPosition doesn't return a copy. It returns the slice of the hidden value.
func (s *Swarm64) IndvSyncUpdatePart3(particleindex int, fitness float64, globalposition []float64) {
	if len(globalposition) != len(s.globalposition) || globalposition == nil {
		globalposition = make([]float64, len(s.globalposition))
		s.mux.RLock()
		copy(globalposition, s.globalposition)
		s.mux.RUnlock()
	}
	s.particles[particleindex].update(s.mode, s.cognative, s.social, s.vmax, s.constriction, globalposition)
}
