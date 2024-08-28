extends RigidBody3D

var screenWidth
var screenHeight

var maxspeed_setting: float # 最快移动速度
var maxAngularVelocity_setting: float # 最快旋转速度
var max_allow_AngularVelocity: float # 允许的最大旋转速度
var turningTime: float # 设计的180度转弯时间
var bursttorque: float # 计算所需的扭矩
var bursttorque_cache: float
var inverttorque: float # 反向扭矩缓存
var maxsacceletation: float # 最大加速度
var threeXspeed: float
var maxsacceletation_cache: float
var breakacceletation: float # 减速G值
var Joystick_deadzone: float
var smoothMouse: float # 鼠标灵敏度

# 玩家状态
var playerHealth: int
var playerMaxHealth: int
var dmgFactor: int = 1
var dmgFactor_cache: int

var previousVelocity: Vector3

var acceletating: bool
var isRotating: bool
var ABS: bool
var On_Destroy: bool
var slowmotion: bool

#onready var headLight = $Body/SpotLight3D
#onready var on_Destroy_StartParticles = $On_Destory_StartParticles
#onready var hitEffect_Particle = $HitEffect_Particle
#onready var explosion_LL = preload("res://path/to/Explosion_LL.ogg")
#onready var lockEnemy = $LockEnemy
#onready var crosshairMouse = $CrosshairMouse
#onready var mouseSensitivity = $MouseSensitivity
#onready var engineSoundController = $EngineSoundController
#onready var healthBar = $HealthBar

func _ready():
	screenWidth = DisplayServer.window_get_size().x
	screenHeight = DisplayServer.window_get_size().y

	# 计算所需的扭矩
	var momentOfInertia = inertia

	ABS = false
	On_Destroy = false
	slowmotion = false

	bursttorque_cache = bursttorque
	maxsacceletation_cache = maxsacceletation

	previousVelocity = linear_velocity

func _physics_process(delta):
	if not On_Destroy:
		apply_movement()
		apply_rotate()
