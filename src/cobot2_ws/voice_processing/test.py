import subprocess
import os

# 재생할 음성 파일 경로 지정
audio_file = "/home/rokey/cobot_ws/src/cobot2_ws/voice_processing/resource/audio/inspect_pass.mp3"

# 시스템 명령어로 재생 실행 (gst-play-1.0 사용)
if os.path.exists(audio_file):
    print("음성 재생 시작...")
    # 재생이 끝날 때까지 파이썬 코드가 대기(Blocking)합니다.
    subprocess.run(["gst-play-1.0", audio_file], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("음성 재생 완료!")
else:
    print(f"오류: 파일을 찾을 수 없습니다. 경로: {audio_file}")