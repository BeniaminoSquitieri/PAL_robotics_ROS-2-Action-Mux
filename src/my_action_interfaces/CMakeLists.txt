cmake_minimum_required(VERSION 3.8)
project(my_action_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genera l'interfaccia per l'azione DoWork
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DoWork.action"
  # Se usi tipi built-in (ad es. builtin_interfaces) aggiungi:
  # DEPENDENCIES builtin_interfaces
)

# Esporta la dipendenza su rosidl_default_runtime
ament_export_dependencies(rosidl_default_runtime)

ament_package()
