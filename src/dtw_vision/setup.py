from setuptools import setup
pkg = 'dtw_vision'
setup(name=pkg, version='0.1.0', packages=[pkg],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + pkg]),
                  ('share/' + pkg, ['package.xml']),
                  ('share/' + pkg + '/models', ['models/ball.sdf'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'ball_tracker = dtw_vision.ball_tracker:main',
          'ball_mover = dtw_vision.ball_mover:main']})
