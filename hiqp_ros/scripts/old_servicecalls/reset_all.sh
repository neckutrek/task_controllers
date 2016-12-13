./remove_all_tasks.sh

sleep .15

./remove_all_primitives.sh

sleep .15

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntconfig1'
type: 'TaskJntConfig'
behaviour: ['DynamicsFirstOrder', '1.0']
priority: 1
visibility: 0
parameters: ['-0.42', '-1.48', '1.21', '0.75', '-0.8', '0.45', '1.21', '0', '0', '0.42', '-1.48', '-1.21', '0.75', '0.8', '0.45', '1.21', '0', '0']"

sleep .15

./remove_all_tasks.sh

sleep .15