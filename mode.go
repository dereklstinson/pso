package pso

//Mode is the Mode flag for the swarm
type Mode int32

//Vanilla sets vanilla Mode
func (m *Mode) Vanilla() Mode { *m = Mode(1); return *m }

//ConstantInertia sets ConstantInertia Mode
func (m *Mode) ConstantInertia() Mode { *m = Mode(2); return *m }

//InertiaReduction sets InertiaReduction Mode
func (m *Mode) InertiaReduction() Mode { *m = Mode(3); return *m }

//Constriction sets Constriction Mode
func (m *Mode) Constriction() Mode { *m = Mode(4); return *m }

//DynamicInertiaMaxVelReduction sets DynamicInertiaMaxVelReduction Mode
func (m *Mode) DynamicInertiaMaxVelReduction() Mode { *m = Mode(5); return *m }

//func (m *Mode) customfunction()Mode{ *m=Mode(6);return *m}
