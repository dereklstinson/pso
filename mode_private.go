package pso

//Mode is the mode flag for the swarm
type mode int32

//Vanilla sets vanilla mode
func (m *mode) vanilla() mode { *m = mode(1); return *m }

//ConstantInertia sets ConstantInertia mode
func (m *mode) constantinertia() mode { *m = mode(2); return *m }

//InertiaReduction sets InertiaReduction mode
func (m *mode) inertiareduction() mode { *m = mode(3); return *m }

//Constriction sets Constriction mode
func (m *mode) constriction() mode { *m = mode(4); return *m }

//DynamicInertiaMaxVelReduction sets DynamicInertiaMaxVelReduction mode
func (m *mode) dynamicInertiaMaxVelReduction() mode { *m = mode(5); return *m }

//func (m *mode) customfunction()mode{ *m=mode(6);return *m}
