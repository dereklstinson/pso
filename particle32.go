package pso

import "math/rand"

type particle struct {
	rng      *rand.Rand
	mode     mode
	source   rand.Source
	fitness  float32
	position []float32
	indvbest []float32
	velocity []float32
	inertia  float32
	alpha    float32
	vmax     float32
}

func createparticle(maxv, minxstart, maxxstart, maxalpha, maxinertia float32, dims int, seed int64, max bool) particle {
	source := rand.NewSource(seed)
	rng := rand.New(source)
	position := make([]float32, dims)
	indvbest := make([]float32, dims)
	velocity := make([]float32, dims)
	var val float32
	for i := range position {
		val = ((maxxstart - minxstart) * rng.Float32()) + minxstart
		position[i] = val
		indvbest[i] = val
		velocity[i] = rng.Float32() * maxv
	}
	var fitness float32
	if max {
		fitness = -9999999
	} else {
		fitness = 9999999
	}
	return particle{
		rng:      rng,
		source:   source,
		fitness:  fitness,
		position: position,
		indvbest: indvbest,
		velocity: velocity,
		inertia:  rng.Float32() * maxinertia,
		alpha:    rng.Float32() * maxalpha,
	}
}
func (p *particle) isbest(fitness float32, max bool) {
	switch max {
	case true:
		if fitness > p.fitness {
			p.fitness = fitness
			copy(p.indvbest, p.position)
		}
	default:
		if fitness < p.fitness {
			p.fitness = fitness
			copy(p.indvbest, p.position)
		}

	}

}

func (p *particle) reset(maxv, minxstart, maxxstart, maxalpha, maxinertia float32) {
	var val float32
	for i := range p.position {
		val = ((maxxstart - minxstart) * p.rng.Float32()) + minxstart
		p.position[i] = val
		p.indvbest[i] = val
		p.velocity[i] = p.rng.Float32() * maxv

	}
	p.alpha = p.rng.Float32() * maxalpha
	p.inertia = p.rng.Float32() * maxinertia
}

//Update will update velocities,and position
func (p *particle) update(mode mode, cognative, social, vmax, constriction float32, globalbest []float32) {
	switch mode {
	case p.mode.vanilla():
		p.vanilla(cognative, social, vmax, globalbest)
	case p.mode.constantinertia():
		p.constant(cognative, social, vmax, globalbest)
	case p.mode.inertiareduction():
		p.linearinertiareduce(cognative, social, vmax, globalbest)
	case p.mode.constriction():
		p.constriction(cognative, social, vmax, constriction, globalbest)
	case p.mode.dynamicInertiaMaxVelReduction():
		p.dimvr(cognative, social, vmax, globalbest)
		//case p.mflg.SocialPressure():
	}

}
func (p *particle) dimvr(cognative, social, vmaxgamma float32, globalbest []float32) {
	min := float32(999999999)
	max := float32(-99999999)
	for i := range p.velocity {
		p.velocity[i] = p.inertia*p.velocity[i] + cognative*p.rng.Float32()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float32()*(globalbest[i]-p.position[i])
		if p.position[i] < min {
			min = p.position[i]
		}
		if p.position[i] > max {
			max = p.position[i]
		}

	}
	vmax := vmaxgamma * (max - min)
	for i := range p.velocity {

		p.velocity[i] = minmagnitudef32(p.velocity[i], vmax)

		p.position[i] += p.velocity[i]
	}
}
func (p *particle) linearinertiareduce(cognative, social, vmax float32, globalbest []float32) {
	for i := range p.velocity {
		p.velocity[i] = (p.alpha * p.inertia * p.velocity[i]) + cognative*p.rng.Float32()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float32()*(globalbest[i]-p.position[i])

		p.velocity[i] = minmagnitudef32(p.velocity[i], vmax)
		p.inertia *= p.alpha
		p.position[i] += p.velocity[i]
	}
}
func (p *particle) vanilla(cognative, social, vmax float32, globalbest []float32) {
	for i := range p.velocity {
		p.velocity[i] += +cognative*p.rng.Float32()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float32()*(globalbest[i]-p.position[i])

		p.velocity[i] = minmagnitudef32(p.velocity[i], vmax)

		p.position[i] += p.velocity[i]
	}
}
func (p *particle) constant(cognative, social, vmax float32, globalbest []float32) {
	for i := range p.velocity {
		p.velocity[i] = (p.inertia * p.velocity[i]) + (cognative * p.rng.Float32() * (p.indvbest[i] - p.position[i])) + (social * p.rng.Float32() * (globalbest[i] - p.position[i]))

		p.velocity[i] = minmagnitudef32(p.velocity[i], vmax)
		p.position[i] += p.velocity[i]
	}
}
func (p *particle) constriction(cognative, social, vmax, constriction float32, globalbest []float32) {
	for i := range p.velocity {
		p.velocity[i] = constriction * (p.velocity[i] + cognative*p.rng.Float32()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float32()*(globalbest[i]-p.position[i]))

		p.velocity[i] = minmagnitudef32(p.velocity[i], vmax)
		p.position[i] += p.velocity[i]
	}
}

func minmagnitudef32(v, vmax float32) float32 {

	if v < 0 {
		if vmax < (-v) {
			return -vmax
		}
		return v

	}
	if vmax < v {
		return vmax
	}
	return v

}
