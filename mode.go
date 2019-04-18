package pso

//Mode is the Mode flag for the swarm
type Mode int32

//Vanilla sets vanilla Mode
func (m *Mode) vanilla() Mode { *m = Mode(1); return *m }

//ConstantInertia sets ConstantInertia Mode
func (m *Mode) constantinertia() Mode { *m = Mode(2); return *m }

//InertiaReduction sets InertiaReduction Mode
func (m *Mode) inertiareduction() Mode { *m = Mode(3); return *m }

//Constriction sets Constriction Mode
func (m *Mode) constriction() Mode { *m = Mode(4); return *m }

//DynamicInertiaMaxVelReduction sets DynamicInertiaMaxVelReduction Mode
func (m *Mode) dynamicInertiaMaxVelReduction() Mode { *m = Mode(5); return *m }

//func (m *Mode) customfunction()Mode{ *m=Mode(6);return *m}
