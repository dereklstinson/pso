package pso

import "math/rand"

type particle64 struct {
	rng      *rand.Rand
	mflg     Mode
	source   rand.Source
	fitness  float64
	position []float64
	indvbest []float64
	velocity []float64
	inertia  float64
	alpha    float64
	vmax     float64
}

func createparticle64(maxv, pminstart, pmaxstart, maxalpha, maxinertia float64, dims int, seed int64, max bool) particle64 {
	source := rand.NewSource(seed)
	rng := rand.New(source)
	position := make([]float64, dims)
	indvbest := make([]float64, dims)
	velocity := make([]float64, dims)
	var val float64
	for i := range position {
		val = (rng.Float64() * (pmaxstart - pminstart)) + pminstart
		position[i] = val
		indvbest[i] = val
		velocity[i] = rng.Float64() * maxv
	}
	var fitness float64
	if max {
		fitness = -9999999
	} else {
		fitness = 9999999
	}
	return particle64{
		rng:      rng,
		source:   source,
		fitness:  fitness,
		position: position,
		indvbest: indvbest,
		velocity: velocity,
		inertia:  rng.Float64() * maxinertia,
		alpha:    rng.Float64() * maxalpha,
	}
}

func (p *particle64) isbest(fitness float64, max bool) {
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
func (p *particle64) reset(maxv, minxstart, maxxstart, maxalpha, maxinertia float64) {
	var val float64
	for i := range p.position {
		val = ((maxxstart - minxstart) * p.rng.Float64()) + minxstart
		p.position[i] = val
		p.indvbest[i] = val
		p.velocity[i] = p.rng.Float64() * maxv

	}
	p.alpha = p.rng.Float64() * maxalpha
	p.inertia = p.rng.Float64() * maxinertia
}

//Update will update velocities,and position
func (p *particle64) update(mode Mode, cognative, social, vmax, constriction float64, globalbest []float64) {
	switch mode {
	case p.mflg.Vanilla():
		p.vanilla(cognative, social, vmax, globalbest)
	case p.mflg.ConstantInertia():
		p.constant(cognative, social, vmax, globalbest)
	case p.mflg.InertiaReduction():
		p.linearinertiareduce(cognative, social, vmax, globalbest)
	case p.mflg.Constriction():
		p.constriction(cognative, social, vmax, constriction, globalbest)
	case p.mflg.DynamicInertiaMaxVelReduction():
		p.dimvr(cognative, social, vmax, globalbest)
		//case p.mflg.SocialPressure():
	}

}
func (p *particle64) dimvr(cognative, social, vmaxgamma float64, globalbest []float64) {
	min := float64(999999999)
	max := float64(-99999999)
	for i := range p.velocity {
		p.velocity[i] = p.inertia*p.velocity[i] + cognative*p.rng.Float64()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float64()*(globalbest[i]-p.position[i])
		if p.position[i] < min {
			min = p.position[i]
		}
		if p.position[i] > max {
			max = p.position[i]
		}

	}
	vmax := vmaxgamma * (max - min)
	for i := range p.velocity {

		p.velocity[i] = minmagnitudef64(p.velocity[i], vmax)

		p.position[i] += p.velocity[i]
	}
}
func (p *particle64) linearinertiareduce(cognative, social, vmax float64, globalbest []float64) {
	for i := range p.velocity {
		p.velocity[i] = (p.inertia * p.velocity[i]) + cognative*p.rng.Float64()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float64()*(globalbest[i]-p.position[i])

		p.velocity[i] = minmagnitudef64(p.velocity[i], vmax)

		p.position[i] += p.velocity[i]
	}
	p.inertia *= p.alpha

}
func (p *particle64) vanilla(cognative, social, vmax float64, globalbest []float64) {
	for i := range p.velocity {
		p.velocity[i] += +cognative*p.rng.Float64()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float64()*(globalbest[i]-p.position[i])

		p.velocity[i] = minmagnitudef64(p.velocity[i], vmax)
		p.position[i] += p.velocity[i]
	}
}
func (p *particle64) constant(cognative, social, vmax float64, globalbest []float64) {
	for i := range p.velocity {
		p.velocity[i] = (p.inertia * p.velocity[i]) + (cognative * p.rng.Float64() * (p.indvbest[i] - p.position[i])) + (social * p.rng.Float64() * (globalbest[i] - p.position[i]))

		p.velocity[i] = minmagnitudef64(p.velocity[i], vmax)
		p.position[i] += p.velocity[i]
	}
}
func (p *particle64) constriction(cognative, social, vmax, constriction float64, globalbest []float64) {
	for i := range p.velocity {
		p.velocity[i] = constriction * (p.velocity[i] + cognative*p.rng.Float64()*(p.indvbest[i]-p.position[i]) + social*p.rng.Float64()*(globalbest[i]-p.position[i]))

		p.velocity[i] = minmagnitudef64(p.velocity[i], vmax)
		p.position[i] += p.velocity[i]
	}
}
func minmagnitudef64(v, vmax float64) float64 {
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
