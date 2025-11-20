import argparse
import math
import sys
import xml.dom.minidom
import json
import os

import packaging.version

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
import rclpy.node
import sensor_msgs.msg
import std_msgs.msg

from .calculate_ur import thetas_to_alphabeta

#########################################################################################
HOPEJR_JOINTS = [
    'shoulder_pitch', 'shoulder_yaw', 'shoulder_roll', 'elbow_flex', 'wrist_roll', 'wrist_yaw',
    'hand_wrist_pitch', 'hand_thumb_mcp_2', 'hand_thumb_pip_1', 'hand_thumb_pip_2', 'hand_thumb_dip',
    'hand_index_mcp_2', 'hand_index_pip', 'hand_index_dip_1',
    'hand_middle_mcp_2', 'hand_middle_pip', 'hand_middle_dip_1',
    'hand_ring_mcp_2', 'hand_ring_pip', 'hand_ring_dip_1',
    'hand_pinky_mcp_2', 'hand_pinky_pip', 'hand_pinky_dip_1',
]

_config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'hopejr_motors.json')
_config_path = os.path.normpath(os.path.abspath(_config_path))
try:
    with open(_config_path, 'r') as _f:
        HOPEJR_MOTORS = json.load(_f)
except Exception as _e:
    HOPEJR_MOTORS = {}
    print(f'Could not load HOPEJR_MOTORS from {_config_path}: {_e}', file=sys.stderr)
##########################################################################################

def _convert_to_float(name, jtype, limit_name, input_string):
    try:
        return float(input_string)
    except ValueError as e:
        raise Exception(
            f'"{limit_name}" limit must be a float for joint "{name}" of type "{jtype}"') from e


class HopejrStatePublisher(rclpy.node.Node):

    def get_param(self, name):
        return self.get_parameter(name).value

    def _init_joint(self, minval, maxval, zeroval):
        joint = {'min': minval, 'max': maxval, 'zero': zeroval}
        if self.pub_def_positions:
            joint['position'] = zeroval
        if self.pub_def_vels:
            joint['velocity'] = 0.0
        if self.pub_def_efforts:
            joint['effort'] = 0.0
        return joint

    def init_sdf(self, xmldom):
        free_joints = {}
        joint_list = []
        dependent_joints = {}

        model_list = xmldom.getElementsByTagName('model')
        if not model_list:
            raise Exception('SDF must have a "model" tag')
        model = model_list[0]
        # Find all non-fixed joints
        for child in model.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName != 'joint':
                continue
            jtype = child.getAttribute('type')
            if jtype in ('gearbox', 'revolute2', 'ball', 'screw', 'universal', 'fixed'):
                continue
            name = child.getAttribute('name')

            # joint limits
            if jtype == 'continuous':
                minval = -math.pi
                maxval = math.pi
            else:
                # Limits are required, and required to be floats.
                limit_list = child.getElementsByTagName('limit')
                if not limit_list:
                    raise Exception(
                        f'Limits must be specified for joint "{name}" of type "{jtype}"')
                limit = limit_list[0]

                lower_list = limit.getElementsByTagName('lower')
                if not lower_list:
                    raise Exception(
                        f'"lower" limit must be specified for joint "{name}" of type "{jtype}"')
                lower = lower_list[0]
                minval = _convert_to_float(name, jtype, 'lower', lower.firstChild.data)

                upper_list = limit.getElementsByTagName('upper')
                if not upper_list:
                    raise Exception(
                        f'"upper" limit must be specified for joint "{name}" of type "{jtype}"')
                upper = upper_list[0]
                maxval = _convert_to_float(name, jtype, 'upper', upper.firstChild.data)

            if self.zeros and name in self.zeros:
                zeroval = self.zeros[name]
            elif minval > 0 or maxval < 0:
                zeroval = (maxval + minval)/2
            else:
                zeroval = 0

            joint = self._init_joint(minval, maxval, zeroval)

            if jtype == 'continuous':
                joint['continuous'] = True
            free_joints[name] = joint
            joint_list.append(name)

        return (free_joints, joint_list, dependent_joints)

    def init_collada(self, xmldom):
        free_joints = {}
        joint_list = []
        dependent_joints = {}

        colladadom = xmldom.childNodes[0]

        if not colladadom.hasAttribute('version'):
            raise Exception('COLLADA must have a version tag')

        colladaversion = packaging.version.parse(colladadom.attributes['version'].value)
        if colladaversion < packaging.version.parse('1.5.0'):
            raise Exception('COLLADA must be at least version 1.5.0')

        kinematics_model_list = xmldom.getElementsByTagName('kinematics_model')
        if not kinematics_model_list:
            raise Exception('COLLADA must have a "kinematics_model" tag')
        kinematics_model = kinematics_model_list[0]
        technique_common_list = kinematics_model.getElementsByTagName('technique_common')
        if not technique_common_list:
            raise Exception('COLLADA must have a "technique_common" tag')
        technique_common = technique_common_list[0]

        for child in technique_common.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName != 'joint':
                continue

            name = child.getAttribute('name')
            revolute_list = child.getElementsByTagName('revolute')
            if not revolute_list:
                continue
            revolute = revolute_list[0]

            limit_list = revolute.getElementsByTagName('limits')
            if not limit_list:
                raise Exception(f'Limits must be specified for joint "{name}" of type "revolute"')

            limit = limit_list[0]

            min_list = limit.getElementsByTagName('min')
            if not min_list:
                raise Exception(
                    f'"min" limit must be specified for joint "{name}" of type "revolute"')
            minval = _convert_to_float(name, 'revolute', 'min',
                                       min_list[0].childNodes[0].nodeValue)

            max_list = limit.getElementsByTagName('max')
            if not max_list:
                raise Exception(
                    f'"max" limit must be specified for joint "{name}" of type "revolute"')
            maxval = _convert_to_float(name, 'revolute', 'max',
                                       max_list[0].childNodes[0].nodeValue)

            if minval == maxval:  # this is a fixed joint
                continue

            joint_list.append(name)
            minval *= math.pi/180.0
            maxval *= math.pi/180.0
            free_joints[name] = self._init_joint(minval, maxval, 0.0)

        return (free_joints, joint_list, dependent_joints)

    def init_urdf(self, xmldom):
        free_joints = {}
        joint_list = []
        dependent_joints = {}

        robot_list = xmldom.getElementsByTagName('robot')
        if not robot_list:
            raise Exception('URDF must have a "robot" tag')
        robot = robot_list[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName != 'joint':
                continue
            jtype = child.getAttribute('type')
            if jtype in ('fixed', 'floating', 'planar'):
                continue
            name = child.getAttribute('name')
            if jtype == 'continuous':
                minval = -math.pi
                maxval = math.pi
            else:
                # Limits are required, and required to be floats.
                limit_list = child.getElementsByTagName('limit')
                if not limit_list:
                    raise Exception(
                        f'Limits must be specified for joint "{name}" of type "{jtype}"')

                limit = limit_list[0]

                if not limit.hasAttribute('lower'):
                    raise Exception(
                        f'"lower" limit must be specified for joint "{name}" of type "{jtype}"')
                minval = _convert_to_float(name, jtype, 'lower', limit.getAttribute('lower'))

                if not limit.hasAttribute('upper'):
                    raise Exception(
                        f'"upper" limit must be specified for joint "{name}" of type "{jtype}"')
                maxval = _convert_to_float(name, jtype, 'upper', limit.getAttribute('upper'))

            safety_tags = child.getElementsByTagName('safety_controller')
            if self.use_small and len(safety_tags) == 1:
                tag = safety_tags[0]
                if tag.hasAttribute('soft_lower_limit'):
                    minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                if tag.hasAttribute('soft_upper_limit'):
                    maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

            joint_list.append(name)

            mimic_tags = child.getElementsByTagName('mimic')
            if self.use_mimic and len(mimic_tags) == 1:
                tag = mimic_tags[0]
                entry = {'parent': tag.getAttribute('joint')}
                if tag.hasAttribute('multiplier'):
                    entry['factor'] = float(tag.getAttribute('multiplier'))
                if tag.hasAttribute('offset'):
                    entry['offset'] = float(tag.getAttribute('offset'))

                dependent_joints[name] = entry
                continue

            if name in dependent_joints:
                continue

            if self.zeros and name in self.zeros:
                zeroval = self.zeros[name]
            elif minval > 0 or maxval < 0:
                zeroval = (maxval + minval)/2
            else:
                zeroval = 0

            joint = self._init_joint(minval, maxval, zeroval)

            if jtype == 'continuous':
                joint['continuous'] = True
            free_joints[name] = joint

        return (free_joints, joint_list, dependent_joints)

    def init_urdf_json(self, xmldom):
        """
        Build free_joints/joint_list/dependent_joints from URDF (init_urdf) AND
        create an internal motor_map describing how HOPEJR_MOTORS controls URDF joints.

        IMPORTANT: do NOT remove or add URDF joint entries. Keep joint_list as the
        canonical list of URDF joints. motor_map stores how motor inputs map into
        URDF joint targets and will be used at publish-time.
        """
        (free_joints, joint_list, dependent_joints) = self.init_urdf(xmldom)

        # prepare motor_map on this node
        self.motor_map = {}  # motor_name -> mapping descriptor
        if not isinstance(HOPEJR_MOTORS, dict):
            return (free_joints, joint_list, dependent_joints)

        urdf_joint_names = set(joint_list)

        for mname, mcfg in HOPEJR_MOTORS.items():
            # skip motors that exactly match an URDF joint name — they are direct
            # (GUI will operate on the URDF joint object).
            if mname in urdf_joint_names:
                # still register default in motor_map so defaults exist
                self.motor_map[mname] = {'type': 'passthrough', 'target': mname,
                                         'factor': 1.0, 'offset': 0.0,
                                         'default': (mcfg.get('default', 0.0) if isinstance(mcfg, dict) else 0.0)}
                continue

            if not isinstance(mcfg, dict):
                # no mapping info -> ignore (could still create a motor_map entry)
                continue

            mapping = mcfg.get('mapping', {})
            # direct mapping to a single URDF joint name
            if 'joint' in mapping:
                tgt = mapping['joint']
                if tgt in urdf_joint_names:
                    self.motor_map[mname] = {
                        'type': 'joint',
                        'target': tgt,
                        'factor': float(mapping.get('factor', 1.0)),
                        'offset': float(mapping.get('offset', 0.0)),
                        'default': float(mcfg.get('default', 0.0))
                    }
                else:
                    # referenced URDF joint missing: log and skip
                    self.get_logger().warn(f"init_urdf_json: motor '{mname}' maps to unknown URDF joint '{tgt}'")
                continue

            # mapping to multiple targets with weights
            if 'targets' in mapping:
                targets = []
                for t in mapping['targets']:
                    jt = t.get('joint')
                    if jt is None:
                        continue
                    w = float(t.get('weight', 1.0))
                    if jt not in urdf_joint_names:
                        self.get_logger().warn(f"init_urdf_json: motor '{mname}' has target '{jt}' not in URDF; skipping that target")
                        continue
                    targets.append({'joint': jt, 'weight': w})
                if targets:
                    self.motor_map[mname] = {
                        'type': 'targets',
                        'targets': targets,
                        'default': float(mcfg.get('default', 0.0))
                    }
                continue

            # If no mapping specified, ignore for now
        # done
        return (free_joints, joint_list, dependent_joints)

    def configure_robot(self, description):
        self.get_logger().debug('Got description, configuring robot')
        xmldom = xml.dom.minidom.parseString(description)

        root = xmldom.documentElement
        if root.tagName == 'sdf':
            (free_joints, joint_list, dependent_joints) = self.init_sdf(xmldom)
        elif root.tagName == 'COLLADA':
            (free_joints, joint_list, dependent_joints) = self.init_collada(xmldom)
        else:
            (free_joints, joint_list, dependent_joints) = self.init_urdf_json(xmldom)

        self.free_joints = free_joints
        self.joint_list = joint_list  # for maintaining the original order of the joints
        self.dependent_joints = dependent_joints

        # Verify that all expected HOPEJR_JOINTS are present in the parsed joint list.
        missing_joints = [j for j in HOPEJR_JOINTS if j not in self.joint_list]
        if missing_joints:
            raise Exception(f'Missing expected joints from robot description: {missing_joints}')

        if self.robot_description_update_cb is not None:
            self.robot_description_update_cb()

    def parse_dependent_joints(self):
        dj = {}
        dependent_joints = self.get_parameters_by_prefix('dependent_joints')
        # get_parameters_by_prefix returned a dictionary of keynames like
        # 'head.parent', 'head.offset', etc. that map to rclpy Parameter values.
        # The rest of the code assumes that the dependent_joints dictionary is
        # a map of name -> dict['parent': parent, factor: factor, offset: offset],
        # where both factor and offset are optional.  Thus we parse the values we
        # got into that structure.
        allowed_joint_names = ('parent', 'factor', 'offset')
        for name, param in dependent_joints.items():
            # First split on the dots; there should be one and exactly one dot
            split = name.split('.')
            if len(split) != 2:
                raise Exception("Invalid dependent_joint name '%s'" % (name))
            newkey = split[0]
            newvalue = split[1]
            if newvalue not in allowed_joint_names:
                allowed_joint_string = ', '.join(f"'{w}'" for w in allowed_joint_names)
                raise Exception("Invalid dependent_joint name '%s' "
                                '(allowed values are %s)' % (newvalue, allowed_joint_string))
            if newkey in dj:
                dj[newkey].update({newvalue: param.value})
            else:
                dj[newkey] = {newvalue: param.value}

        # Now ensure that there is at least a 'parent' in all keys
        for name, outdict in dj.items():
            if outdict.get('parent', None) is None:
                raise Exception('All dependent_joints must at least have a parent')

        return dj

    def declare_ros_parameter(self, name, default, descriptor):
        # When the automatically_declare_parameters_from_overrides parameter to
        # rclpy.create_node() is True, then any parameters passed in on the
        # command-line are automatically declared.  In that case, calling
        # node.declare_parameter() will raise an exception.  However, in the case
        # where a parameter is *not* overridden, we still want to declare it
        # (so it shows up in "ros2 param list", for instance).  Thus we always do
        # a declaration and just ignore ParameterAlreadyDeclaredException.

        try:
            self.declare_parameter(name, default, descriptor)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

    def robot_description_cb(self, msg):
        try:
            self.configure_robot(msg.data)
        except Exception as e:
            self.get_logger().warn(str(e))

    def __init__(self, description_file):
        super().__init__('hopejr_state_publisher',
                         automatically_declare_parameters_from_overrides=True)

        self.declare_ros_parameter('publish_default_efforts', False,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('publish_default_positions', True,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('publish_default_velocities', False,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('rate', 10,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_ros_parameter('source_list', [],
                                   ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_ros_parameter('use_mimic_tags', True,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('use_smallest_joint_limits', True,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_ros_parameter('delta', 0.0,
                                   ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        # In theory we would also declare 'dependent_joints' and 'zeros' here.
        # Since rclpy doesn't support maps natively, though, we just end up
        # letting 'automatically_declare_parameters_from_overrides' declare
        # any parameters for us.

        self.free_joints = {}
        self.joint_list = []  # for maintaining the original order of the joints
        self.dependent_joints = self.parse_dependent_joints()
        # store runtime motor-only values set by GUI sliders (motor_name -> value)
        # GUI will write into this dict for motors that do not correspond to URDF free_joints.
        self.motors = {}
        self.use_mimic = self.get_param('use_mimic_tags')
        self.use_small = self.get_param('use_smallest_joint_limits')

        zeros = self.get_parameters_by_prefix('zeros')
        # get_parameters_by_prefix() returns a map of name -> Parameter
        # structures, but self.zeros is expected to be a list of name -> float;
        # fix that here.
        self.zeros = {k: v.value for (k, v) in zeros.items()}

        self.pub_def_positions = self.get_param('publish_default_positions')
        self.pub_def_vels = self.get_param('publish_default_velocities')
        self.pub_def_efforts = self.get_param('publish_default_efforts')

        self.robot_description_update_cb = None

        if description_file is not None:
            # If we were given a URDF file on the command-line, use that.
            with open(description_file, 'r') as infp:
                description = infp.read()
            self.configure_robot(description)
        else:
            self.get_logger().info(
                'Waiting for robot_description to be published on the robot_description topic...')

        # In all cases, subscribe to the '/robot_description' topic; this allows us to get our
        # initial configuration in the case we weren't given it on the command-line, and allows
        # us to dynamically update later.
        qos = rclpy.qos.QoSProfile(depth=1,
                                   durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(std_msgs.msg.String,
                                 'robot_description',
                                 lambda msg: self.robot_description_cb(msg),
                                 qos)

        self.delta = self.get_param('delta')

        source_list = self.get_param('source_list')
        self.sources = []
        for source in source_list:
            self.sources.append(self.create_subscription(sensor_msgs.msg.JointState, source,
                                                         self.source_cb, 10))

        # The source_update_cb will be called at the end of self.source_cb.
        # The main purpose is to allow external observers (such as the
        # hopejr_state_publisher_gui) to be notified when things are updated.
        self.source_update_cb = None

        self.pub = self.create_publisher(sensor_msgs.msg.JointState, 'joint_states', 10)

        self.timer = self.create_timer(1.0 / self.get_param('rate'), self.timer_callback)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.source_update_cb is not None:
            self.source_update_cb()

    def set_source_update_cb(self, user_cb):
        self.source_update_cb = user_cb

    def set_robot_description_update_cb(self, user_cb):
        self.robot_description_update_cb = user_cb

    def timer_callback(self):
        # Publish Joint States
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.delta > 0:
            self.update(self.delta)

        # --- gather motor input values ---
        # priority: 1) self.free_joints[name]['position'] (if URDF joint exists)
        #           2) self.motors[name] (set by GUI sliders for motor-only entries)
        #           3) HOPEJR_MOTORS config default
        motor_values = {}
        flexor_buffer = {}

        for mname, mcfg in (HOPEJR_MOTORS.items() if isinstance(HOPEJR_MOTORS, dict) else []):
            val = 0.0
            # 1) URDF-backed joint current position
            if hasattr(self, 'free_joints') and mname in self.free_joints and 'position' in self.free_joints[mname]:
                try:
                    val = float(self.free_joints[mname]['position'])
                    motor_values[mname] = val
                    continue
                except Exception:
                    pass
            # 2) GUI-provided motor-only value
            if hasattr(self, 'motors') and mname in self.motors:
                try:
                    val = float(self.motors[mname])
                    motor_values[mname] = val
                    continue
                except Exception:
                    pass
            # 3) config default
            if isinstance(mcfg, dict):
                try:
                    val = float(mcfg.get('default', 0.0))
                except Exception:
                    val = 0.0
            motor_values[mname] = val

        # --- compute mapped_positions for URDF joints ---
        mapped_positions = {}  # urdf_joint_name -> position value

        def _clamp(x, a=-1.0, b=1.0):
            return max(a, min(b, x))

        for mname, mcfg in (HOPEJR_MOTORS.items() if isinstance(HOPEJR_MOTORS, dict) else []):
            mval = float(motor_values.get(mname, 0.0))
            mapping = mcfg.get('mapping', {}) if isinstance(mcfg, dict) else {}

            # Special thumb formula when motor name startswith 'thumb_'
            if mname.startswith('thumb_') and mname != 'thumb_cmc':
                # compute thumb_val = (pi/2) - arccos(3.6/7.6) + arccos((3.6+4.5*theta)/7.6)
                theta = float(mval)
                a1 = _clamp(3.6 / 7.6)
                a2 = _clamp((3.6 + 4.5 * theta) / 7.6)
                try:
                    thumb_val = math.acos(a1) - math.acos(a2)
                except ValueError:
                    # numeric safety fallback
                    thumb_val = 0.0
                # apply thumb_val to mapping targets or single joint
                if isinstance(mapping, dict) and 'joint' in mapping:
                    tgt = mapping['joint']
                    mapped_positions[tgt] = thumb_val
                elif isinstance(mapping, dict) and 'targets' in mapping:
                    for t in mapping['targets']:
                        tgt = t.get('joint')
                        weight = float(t.get('weight', 1.0))
                        mapped_positions[tgt] = mapped_positions.get(tgt, 0.0) + thumb_val * weight
                # done with this motor
                continue

            # Special handling for motors ending with '_pip_dip'
            if mname.endswith('_pip_dip'):
                theta = float(mval)
                try:
                    pip_val = math.acos(2.6 / 7.6) - math.acos((2.6 + 2.25 * theta) / 7.6)
                except ValueError:
                    pip_val = 0.0  # Fallback for numeric safety
                # apply pip_val to mapping targets or single joint
                if isinstance(mapping, dict) and 'joint' in mapping:
                    tgt = mapping['joint']
                    mapped_positions[tgt] = pip_val
                elif isinstance(mapping, dict) and 'targets' in mapping:
                    for t in mapping['targets']:
                        tgt = t.get('joint')
                        weight = float(t.get('weight', 1.0))
                        mapped_positions[tgt] = mapped_positions.get(tgt, 0.0) + pip_val * weight
                # done with this motor
                continue

            # Special handling for motors ending with '_flexor'
            if mname.endswith('_flexor'):
                parts = mname.split('_')
                if len(parts) < 3:
                    continue
                subtype = parts[-2]
                if subtype not in ('radial', 'ulnar'):
                    continue
                finger = '_'.join(parts[:-2])
                joint_prefix = finger if finger.startswith('hand_') else f'hand_{finger}'
                entry = flexor_buffer.setdefault(joint_prefix, {})
                entry[subtype] = float(mval)
                if 'radial' in entry and 'ulnar' in entry:
                    try:
                        alpha, beta = thetas_to_alphabeta(entry['radial'], entry['ulnar'])
                    except ValueError:
                        alpha, beta = 90.0, 0.0  # numeric safety
                    mapped_positions[f'{joint_prefix}_mcp_2'] = alpha
                    mapped_positions[f'{joint_prefix}_pip'] = beta / 2.0
                continue

            # Non-thumb default handling
            if isinstance(mapping, dict) and 'joint' in mapping:
                tgt = mapping['joint']
                factor = float(mapping.get('factor', 1.0))
                offset = float(mapping.get('offset', 0.0))
                # Ensure that we are using the motor value directly for direct mappings
                if 'description' in mcfg and 'Direct map' in mcfg['description']:
                    if mname == tgt:  # Only use motor value directly if names match
                        mapped_positions[tgt] = mval  # Use motor value directly for direct mappings
                else:
                    mapped_positions[tgt] = mval * factor + offset
            elif isinstance(mapping, dict) and 'targets' in mapping:
                for t in mapping['targets']:
                    tgt = t.get('joint')
                    weight = float(t.get('weight', 1.0))
                    mapped_positions[tgt] = mapped_positions.get(tgt, 0.0) + mval * weight
            # else: no mapping -> nothing to do for this motor

        # --- derive mapped values for dependent (mimic) joints ---
        # For each dependent joint, follow the mimic chain up to a free joint and
        # compute: value = parent_value * (f1*f2*...) + (o1 + f1*o2 + f1*f2*o3 + ...)
        if hasattr(self, 'dependent_joints') and isinstance(self.dependent_joints, dict):
            for dep_name in list(self.dependent_joints.keys()):
                seen = set()
                f_total = 1.0
                o_total = 0.0
                current = dep_name
                # first step: get immediate parent of dep_name
                param = self.dependent_joints.get(current)
                if not param:
                    continue
                parent = param.get('parent')
                # accumulate for first relation
                f = float(param.get('factor', 1.0))
                o = float(param.get('offset', 0.0))
                o_total = o * f_total + o_total
                f_total *= f
                current = parent
                # walk up any further mimic chain
                while current in self.dependent_joints:
                    if current in seen:
                        self.get_logger().error(f'Infinite mimic chain detected at "{current}" for dependent "{dep_name}"')
                        break
                    seen.add(current)
                    pparam = self.dependent_joints[current]
                    pf = float(pparam.get('factor', 1.0))
                    po = float(pparam.get('offset', 0.0))
                    o_total = po * f_total + o_total
                    f_total *= pf
                    current = pparam.get('parent')
                # current should now be the ultimate free joint name
                if current not in self.free_joints:
                    # Can't resolve parent to a free joint; skip
                    self.get_logger().debug(f'Could not resolve dependent "{dep_name}" parent "{current}" to a free joint')
                    continue
                # base parent value prefers any mapped override
                base_parent_val = mapped_positions.get(current, self.free_joints[current].get('position', self.free_joints[current].get('zero', 0.0)))
                try:
                    dep_val = float(base_parent_val) * f_total + o_total
                    mapped_positions[dep_name] = dep_val
                except Exception:
                    # ignore numeric errors for safety
                    self.get_logger().warn(f'Failed to compute mimic value for "{dep_name}"')
                    continue

        # --- Initialize msg arrays and fill, using mapped_positions overrides ---
        has_position = len(self.dependent_joints.items()) > 0
        has_velocity = False
        has_effort = False
        for name, joint in self.free_joints.items():
            if not has_position and 'position' in joint:
                has_position = True
            if not has_velocity and 'velocity' in joint:
                has_velocity = True
            if not has_effort and 'effort' in joint:
                has_effort = True
        num_joints = (len(self.free_joints.items()) + len(self.dependent_joints.items()))
        if has_position:
            msg.position = num_joints * [0.0]
        if has_velocity:
            msg.velocity = num_joints * [0.0]
        if has_effort:
            msg.effort = num_joints * [0.0]

        for i, name in enumerate(self.joint_list):
            msg.name.append(str(name))
            joint = None

            # Free joint
            if name in self.free_joints:
                joint = self.free_joints[name]
                factor = 1
                offset = 0
            # Dependent joint (mimic)
            elif name in self.dependent_joints:
                param = self.dependent_joints[name]
                parent = param['parent']
                factor = param.get('factor', 1.0)
                offset = param.get('offset', 0.0)
                recursive_mimic_chain_joints = [name]
                while parent in self.dependent_joints:
                    if parent in recursive_mimic_chain_joints:
                        error_message = 'Found an infinite recursive mimic chain'
                        self.get_logger().error(f'{error_message}: {recursive_mimic_chain_joints + [parent]}')
                        sys.exit(1)
                    recursive_mimic_chain_joints.append(parent)
                    param = self.dependent_joints[parent]
                    parent = param['parent']
                    offset += factor * param.get('offset', 0)
                    factor *= param.get('factor', 1)
                joint = self.free_joints[parent]

            if has_position and 'position' in joint:
                # override from mapped_positions if present (mapped_positions keys are URDF joint names)
                if name in mapped_positions:
                    msg.position[i] = float(mapped_positions[name])
                else:
                    msg.position[i] = float(joint['position']) * factor + offset
            if has_velocity and 'velocity' in joint:
                msg.velocity[i] = float(joint['velocity']) * factor
            if has_effort and 'effort' in joint:
                msg.effort[i] = float(joint['effort'])

        if msg.name or msg.position or msg.velocity or msg.effort:
            # Only publish non-empty messages
            self.pub.publish(msg)

    def update(self, delta):
        for name, joint in self.free_joints.items():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward


def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'description_file', help='Robot description file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])
    jsp = HopejrStatePublisher(parsed_args.description_file)

    try:
        rclpy.spin(jsp)
    except KeyboardInterrupt:
        pass

    jsp.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()