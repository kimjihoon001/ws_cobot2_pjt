# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
import subprocess
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate  # d2 이거를 langchain_core로 바꿈
# from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from od_msg.srv import ListenYesNo
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############

#----------------------------------------------------------------
# current_dir = os.getcwd()
# package_path = get_package_share_directory("pick_and_place_voice")

# env_path = "/home/rokey/cobot_ws/src/cobot2_ws/pick_and_place_voice/resource/.env"
# load_dotenv(dotenv_path=env_path)
# is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
# openai_api_key = os.getenv("OPENAI_API_KEY")
#-----------------------------------------------------------------

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

# 제품별로 필요한 도구 매핑 (LLM이 아니라 코드가 확실하게 처리하기 위한 딕셔너리)
# 기본형: 젠가 누락 없음 -> 도구 필요 없음. A/B/C형은 각각 누락 패턴이 다른 불량품 유형.
PRODUCT_TOOL_MAP = {
    "기본형": [],
    "A형": ["hammer", "screwdriver"],
    "B형": ["hammer"],
    "C형": ["screwdriver"],
}

# 확인/릴리즈 응답을 음성으로도 받기 위한 예/아니오 키워드 (docs/voice_hmi_workflow.md 설계안 그대로)
YES_WORDS = ["응", "맞아", "네", "그래", "오케이", "맞습니다", "그거 맞아", "yes", "ok"]
NO_WORDS = ["아니", "아니야", "틀렸어", "다시", "그거 아니야", "잘못됐어", "no", "cancel"]

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 로봇에게 명령을 전달하기 위해, 사용자의 음성 명령에서 도구와 목적지만 정확히 추출하는 파서(parser)입니다.
            인사말, 설명, 부연 설명 없이 오직 아래 출력 형식으로만 답하세요.

            <도구 목록>
            hammer, screwdriver

            <목적지 목록>
            pos1, pos2, pos3, user

            <제품별 필요 도구 매핑>
            기본형: (도구 없음)
            A형: hammer, screwdriver
            B형: hammer
            C형: screwdriver
            (목록에 없는 제품명은 매핑하지 말고 도구 없음으로 처리하세요.)

            <규칙>
            1. 문장에서 <도구 목록>에 있는 도구와 각 도구의 목적지를 등장 순서대로 추출하세요.
            2. 명확한 도구 명칭이 없어도 문맥상 유추 가능하면(예: "망치", "못 박는 것" → hammer /
               "나사 돌리는 것", "드라이버" → screwdriver) 추론해서 반환하세요.
            3. "A형"/"B형"/"C형"/"기본형"처럼 제품명으로 요청하면 <제품별 필요 도구 매핑>의 도구를
               전부 출력하세요 (기본형은 도구 없음 → 빈 문자열).
            4. 목적지를 말하지 않았으면 모든 도구의 목적지를 user로 채우고, 명시했으면 그 목적지를 쓰세요.

            <출력 형식>
            "도구1 도구2 ... / 목적지1 목적지2 ..."
            - 도구/목적지는 공백으로 구분, 순서는 서로 대응
            - 도구가 없으면 '/' 앞을, 목적지가 없으면 '/' 뒤를 비웁니다 (공백 없이)

            <예시>
            입력: "hammer를 user에게 가져다 놔"
            출력: hammer / user

            입력: "왼쪽에 있는 망치를 작업대에 넣어줘"
            출력: hammer / user

            입력: "screwdriver 좀 줘"
            출력: screwdriver /

            입력: "A형 상품 도구 가져다줘"
            출력: hammer screwdriver / user user

            입력: "B형 상품 도구 가져다줘"
            출력: hammer / user

            입력: "C형 상품 도구 가져다줘"
            출력: screwdriver / user

            입력: "기본형 상품이야"
            출력: /

            입력: "안녕, 오늘 날씨 좋다"
            출력: /

            <사용자 입력>
            "{user_input}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        # self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=0,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request... ('hello rokey' wakeup word active)")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        # 음성으로 "네/아니오"를 확인하는 것도 다른 노드(tool_pick_yolo_target.py의
        # 배송 완료 확인)에서 재사용할 수 있도록 서비스로 노출
        self.listen_confirmation_srv = self.create_service(
            ListenYesNo, 'listen_confirmation', self._handle_listen_confirmation
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)
        self.play_audio("sys_start.mp3")

    def play_audio(self, filename):
        audio_path = os.path.join(RESOURCE_PATH, f"audio/{filename}")
        if os.path.exists(audio_path):
            subprocess.run(["gst-play-1.0", audio_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def extract_keyword(self, output_message):  # d2 이 함수 일부 수정함
        """(tools, targets, matched_product) 반환. matched_product=True는 제품명이
        PRODUCT_TOOL_MAP으로 명시적으로 인식된 경우(기본형처럼 도구가 0개인 것도 포함)이고,
        False는 LLM 폴백 결과라는 뜻 — get_keyword()가 "도구 0개=진짜 실패"와
        "도구 0개=기본형이라 원래 없음"을 구분하는 데 사용."""
        # LLM한테 맡기기 전에, 코드가 먼저 product 언급 여부를 확인해서
        # 확실한 매핑이 있으면 그걸로 바로 반환 (LLM이 매핑을 잘못 기억할 위험 자체를 없앰)
        for product_name, tools in PRODUCT_TOOL_MAP.items():
            if product_name in output_message:
                targets = ["user"] * len(tools)  # product로 부르면 항상 user한테 가져다 놓음
                print(f"'{product_name}' 감지됨 -> 매핑된 도구로 바로 반환: {tools} / {targets}")
                return tools, targets, True

        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object} / {target}")
        return object, target, False

    def listen_yes_no(self, timeout_sec=5.0):
        """짧게 STT 1회로 "네/아니오"류 답변인지 키워드로 판별.
        반환: (heard, confirmed) - heard=False면 명확한 답을 못 들은 것(애매함/무관한 말)."""
        original_duration = self.stt.duration
        self.stt.duration = timeout_sec
        try:
            text = self.stt.speech2text()
        finally:
            self.stt.duration = original_duration

        if any(word in text for word in YES_WORDS):
            return True, True
        if any(word in text for word in NO_WORDS):
            return True, False
        return False, False

    def _handle_listen_confirmation(self, request, response):
        """다른 노드(tool_pick_yolo_target.py의 배송 완료 확인 등)가 재사용하는 서비스."""
        heard, confirmed = self.listen_yes_no()
        response.heard = heard
        response.confirmed = confirmed
        return response

    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함    # d2 이 함수 일부 수정함
        self.play_audio("sys_wait.mp3")
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
            print("'hello rokey' 호출 대기 중...")
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        tools, targets, matched_product = self.extract_keyword(output_message)

        if not tools:
            if matched_product:
                # 기본형처럼 "도구가 원래 필요 없는 제품"으로 명시 인식된 경우 - 정상 완료 처리
                self.get_logger().info("도구가 필요 없는 제품으로 인식됨 (예: 기본형)")
                self.play_audio("no_tool_needed.mp3")
                response.success = True
                response.message = ""
                return response
            # 아무것도 인식 못했으면 그대로 종료
            response.success = False
            response.message = ""
            return response

        # 확인 절차 없이 바로 확정 - 음성으로 듣고 곧장 픽업으로 넘어간다.
        self.get_logger().warn(f"Detected tools: {tools} / {targets}")
        self.play_audio("order_recv.mp3")
        response.success = True
        # 도구:목적지 쌍으로 실어서 반환
        response.message = " ".join(f"{t}:{d}" for t, d in zip(tools, targets))
        return response


def main():  # d2 메인문 일부 수정
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
