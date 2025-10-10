# Dependency Graph Analysis for Heterogeneous USV-UAV Mission Coordination

## Detailed Description for Research Paper

### Paragraph 1: Dependency Graph Architecture and Construction

The dependency graph in our heterogeneous USV-UAV mission coordination system represents a directed acyclic graph (DAG) where nodes correspond to individual symbolic actions τᵢ = aᵢ(robot, {θᵢ}, {σᵢ}) and edges encode temporal and spatial constraints between mission steps. The graph construction algorithm analyzes the generated mission plan and automatically identifies three primary types of dependencies: (1) **Inter-robot spatial dependencies**, where UAV actions such as Takeoff(UAV, {USVDeck, alt=15m}, {}) require the USV to be positioned at a specific location through a preceding Navigate(USV, {position}, {}) action; (2) **Sequential robot-specific dependencies**, ensuring that each robot completes its current action before proceeding to the next, maintaining state consistency and preventing command conflicts; and (3) **Mission-critical synchronization points**, such as LandOnUSV(UAV, {USVDeck}, {sensors, safety_checks}) which mandates that the USV maintains a stable platform position while the UAV approaches for landing. The dependency resolution mechanism employs a topological sorting approach combined with real-time status monitoring, where each completed action τᵢ triggers a re-evaluation of all pending actions to identify newly executable steps whose dependency prerequisites have been satisfied.

### Paragraph 2: Dynamic Execution and Coordination Logic

The mission coordinator implements a dynamic execution engine that continuously monitors the dependency graph state and executes all ready actions in parallel when their dependencies are satisfied, maximizing system efficiency while maintaining safety constraints. The coordination logic maintains a completion tracking system using a set-based approach where completed_steps = {i | τᵢ ∈ executed_actions}, and for each pending action τⱼ, the system verifies that all dependencies deps(j) ⊆ completed_steps before initiating execution. This approach enables sophisticated coordination patterns such as allowing the UAV to perform Survey(UAV, {Orbit360, alt=15m}, {sensors=cam}) while the USV simultaneously executes Navigate(USV, {next_position}, {}) to the next waypoint, provided there are no spatial conflicts. The dependency graph also handles error recovery and mission replanning scenarios, where failed actions can trigger dependency re-evaluation and alternative execution paths. Critical synchronization points, such as the transition from autonomous UAV operations back to USV-deck landing, are managed through explicit dependency chains that ensure proper sequencing of FlyTo(UAV, {HoverPointAboveUSV}, {}) → LandOnUSV(UAV, {USVDeck}, {safety_checks}) → GoHome(USV, {PortDock}, {}), guaranteeing mission safety and operational integrity throughout the entire heterogeneous system coordination process.

## Visual Dependency Graph

```
Mission: Port Crane Inspection with USV-UAV Coordination

Actions:
τ₀: Navigate(USV, {pos:(1400,20), orient:90°}, {})
τ₁: Takeoff(UAV, {USVDeck, alt:15m}, {})  
τ₂: FlyTo(UAV, {pos:(1400,20), alt:15m}, {})
τ₃: Survey(UAV, {Orbit360, alt:15m, dur:300s}, {sensors:[cam], detect:anomaly})
τ₄: Report(UAV, {}, {detect:anomaly, conf:0.82})
τ₅: FlyTo(UAV, {HoverAboveUSV}, {})
τ₆: LandOnUSV(UAV, {USVDeck}, {safety_checks:true})
τ₇: GoHome(USV, {pos:(0,0)}, {})

Dependency Graph:
        ┌─────┐
        │ τ₀  │ (USV Navigate to position)
        │ USV │
        └──┬──┘
           │ spatial dependency
           ▼
        ┌─────┐     sequential     ┌─────┐     sequential     ┌─────┐
        │ τ₁  │────────────────────▶│ τ₂  │────────────────────▶│ τ₃  │
        │ UAV │    (same robot)    │ UAV │    (same robot)    │ UAV │
        └─────┘                    └─────┘                    └──┬──┘
                                                                 │ mission flow
                                                                 ▼
        ┌─────┐     sequential     ┌─────┐     spatial        ┌─────┐     spatial      ┌─────┐
        │ τ₇  │◀───────────────────│ τ₆  │◀──────────────────│ τ₅  │◀─────────────────│ τ₄  │
        │ USV │    (mission end)   │ UAV │   (USV platform)  │ UAV │   (return path)  │ UAV │
        └─────┘                    └─────┘                    └─────┘                  └─────┘

Dependencies:
deps(τ₁) = {τ₀}     // Takeoff depends on USV positioning
deps(τ₂) = {τ₁}     // Sequential UAV actions
deps(τ₃) = {τ₂}     // Sequential UAV actions  
deps(τ₄) = {τ₃}     // Report after survey
deps(τ₅) = {τ₄}     // Return after report
deps(τ₆) = {τ₅}     // Land after hover positioning
deps(τ₇) = {τ₆}     // Go home after UAV landing

Execution Timeline:
t₀: Execute τ₀ (USV Navigate)
t₁: Complete τ₀ → Execute τ₁ (UAV Takeoff)  
t₂: Complete τ₁ → Execute τ₂ (UAV FlyTo)
t₃: Complete τ₂ → Execute τ₃ (UAV Survey) 
t₄: Complete τ₃ → Execute τ₄ (UAV Report)
t₅: Complete τ₄ → Execute τ₅ (UAV Return)
t₆: Complete τ₅ → Execute τ₆ (UAV Land)
t₇: Complete τ₆ → Execute τ₇ (USV Home)

Parallelization Opportunities:
- τ₃ (Survey) can run while USV maintains position
- Independent actions with no spatial conflicts can execute simultaneously
- Status monitoring enables real-time dependency resolution
```

## Mathematical Representation

```
Dependency Graph G = (V, E) where:

V = {τᵢ | i ∈ [0, n-1]} (set of mission actions)
E = {(τᵢ, τⱼ) | τⱼ depends on τᵢ} (dependency edges)

Dependency Function:
deps: V → P(V)
deps(τⱼ) = {τᵢ ∈ V | (τᵢ, τⱼ) ∈ E}

Execution Condition:
executable(τⱼ) ⟺ deps(τⱼ) ⊆ completed_steps

Topological Ordering:
∀(τᵢ, τⱼ) ∈ E: exec_time(τᵢ) < exec_time(τⱼ)
```

## Implementation Features

1. **Real-time Dependency Resolution**: Continuous monitoring and dynamic execution
2. **Parallel Execution**: Multiple robots can execute independent actions simultaneously  
3. **Safety Constraints**: Critical dependencies prevent unsafe operations
4. **Error Recovery**: Failed actions trigger dependency re-evaluation
5. **Scalability**: Graph structure scales with mission complexity

This dependency graph architecture ensures safe, efficient, and coordinated execution of complex heterogeneous missions while maintaining the flexibility to adapt to dynamic operational conditions.
