from setuptools import setup
setup(name='clearing_robot_drl',version='0.1.0',packages=['clearing_robot_drl'],
      data_files=[('share/ament_index/resource_index/packages',['resource/clearing_robot_drl']),('share/clearing_robot_drl',['package.xml','README.md'])],
      install_requires=['setuptools','numpy','torch','tensorboard'],zip_safe=False,
      entry_points={'console_scripts':['clearing_drl=clearing_robot_drl.runtime:main','clearing_drl_export=clearing_robot_drl.export:main']})
