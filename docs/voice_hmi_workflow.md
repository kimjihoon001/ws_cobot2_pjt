# Voice Processing - HMI 연결 워크플로우 정리

작성일: 2026-07-07

## 오늘 구현한 것

오늘 구현한 핵심은 **음성 명령 결과를 바로 로봇 작업으로 넘기지 않고, Web HMI 작업 화면에서 한 번 더 확인받는 흐름**이다.

예를 들어 사용자가 음성으로 이렇게 말한다.

```text
해머 가져다줘
```

그러면 기존에는 `voice_processing`이 음성을 텍스트로 바꾸고, 도구 이름을 뽑은 뒤 바로 `/get_keyword` 응답으로 넘기는 구조였다.

오늘 만든 구조에서는 중간에 HMI 확인 단계가 추가됐다.

```text
사용자 음성
-> STT
-> LLM/파서로 도구와 목적지 추출
-> HMI 작업 화면에 "이 도구가 맞나요?" 표시
-> 사람이 HMI에서 확인/아니오 선택
-> 확인이면 로봇 제어 쪽으로 도구 목록 반환
-> 아니오면 다시 음성 입력 받기
```

즉 오늘 한 일은 간단히 말하면 **voice_processing과 Web HMI 사이에 확인용 다리(bridge)를 놓은 것**이다.

## 전체 구조

현재 관련 컴포넌트는 크게 3개다.

1. `voice_processing/get_keyword.py`

음성 입력을 처리하는 ROS2 노드다.

- 웨이크워드 감지
- STT로 음성을 텍스트로 변환
- LLM 또는 코드 매핑으로 도구/목적지 추출
- HMI에 확인 요청
- HMI 응답을 기다렸다가 `/get_keyword` 응답 반환

관련 파일:

```text
/home/rokey/Downloads/ws_cobot2_pjt-voice_process/src/cobot2_ws/voice_processing/voice_processing/get_keyword.py
```

2. HMI 백엔드 `ros_bridge.py`

FastAPI 서버 안에서 같이 도는 ROS2 브릿지 노드다.

이 노드는 `/confirm_tools`라는 ROS2 서비스를 제공한다.

`get_keyword.py`가 `/confirm_tools` 서비스를 호출하면, 백엔드는 그 요청을 WebSocket 메시지로 바꿔서 브라우저 HMI로 보낸다.

관련 파일:

```text
/home/rokey/ws_cobot2_pjt-web-hmi/backend/app/ros_bridge.py
```

3. HMI 프론트 `WorkSessionPage`

브라우저에서 보는 작업 화면이다.

백엔드 WebSocket에서 `confirm_request` 메시지를 받으면 좌측 `현재 작업` 카드에 도구와 목적지를 표시하고, `확인` / `아니오` 버튼을 보여준다.

관련 파일:

```text
/home/rokey/ws_cobot2_pjt-web-hmi/frontend/src/hooks/useVoiceBridge.ts
/home/rokey/ws_cobot2_pjt-web-hmi/frontend/src/pages/WorkSessionPage.tsx
```

## ROS2 서비스 연결

이번 연결의 핵심 서비스는 `ConfirmTools`다.

서비스 정의:

```text
string[] tools
string[] targets
---
bool confirmed
```

의미는 이렇다.

```text
request:
  tools: ["hammer", "screwdriver"]
  targets: ["user", "user"]

response:
  confirmed: true 또는 false
```

`tools`는 감지된 도구 목록이고, `targets`는 각 도구를 어디로 가져갈지 나타내는 목적지 목록이다.

예를 들어:

```text
hammer -> user
screwdriver -> user
```

이런 식으로 HMI에 표시된다.

## 실제 동작 흐름

### 1. 로봇 제어 쪽에서 `/get_keyword` 호출

로봇 제어 노드나 터미널에서 다음 서비스가 호출된다.

```bash
ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
```

### 2. `get_keyword.py`가 음성을 듣는다

`get_keyword.py`는 웨이크워드를 기다린다.

```text
hello rokey
```

이후 5초 정도 음성을 녹음하고 STT를 수행한다.

### 3. STT 결과에서 도구/목적지를 뽑는다

예:

```text
STT 결과: 해머랑 드라이버 모두 가져다줘
LLM response: ["hammer", "screwdriver"] / ["user", "user"]
```

### 4. `get_keyword.py`가 HMI에 확인 요청

`get_keyword.py`는 바로 성공 응답을 주지 않고, `/confirm_tools` 서비스를 호출한다.

```python
req.tools = tools
req.targets = targets
future = self.confirm_client.call_async(req)
```

여기서 `get_keyword.py`는 HMI의 답을 기다린다.

### 5. HMI 백엔드가 요청을 받음

HMI 백엔드의 `VoiceBridgeNode`가 `/confirm_tools` 요청을 받는다.

받은 요청을 이런 WebSocket 메시지로 바꾼다.

```json
{
  "type": "confirm_request",
  "tools": ["hammer", "screwdriver"],
  "targets": ["user", "user"]
}
```

### 6. 브라우저 작업 화면에 표시

프론트의 `useVoiceBridge()`가 WebSocket 메시지를 받고 `pending` 상태를 채운다.

그러면 `WorkSessionPage`에서 다음처럼 보인다.

```text
확인 필요
hammer -> user, screwdriver -> user

[확인] [아니오]
```

### 7. 작업자가 확인/아니오 클릭

작업자가 `확인`을 누르면 프론트가 백엔드로 다음 메시지를 보낸다.

```json
{
  "cmd": "confirm_response",
  "confirmed": true
}
```

`아니오`를 누르면:

```json
{
  "cmd": "confirm_response",
  "confirmed": false
}
```

### 8. 백엔드가 ROS2 서비스 응답 반환

백엔드는 이 값을 `/confirm_tools` 서비스 응답으로 돌려준다.

```text
confirmed=True
```

### 9. `get_keyword.py`가 최종 처리

확인이 true면:

```text
response.success = True
response.message = "hammer:user screwdriver:user"
```

아니오면:

```text
다시 음성 입력을 받음
```

## 오늘 겪은 문제

문제는 크게 두 가지였다.

## 문제 1. HMI 백엔드를 `uvicorn --reload`로 실행함

처음에는 HMI 백엔드를 이렇게 실행했다.

```bash
python3 -m uvicorn app.main:app --reload --port 8000
```

일반 FastAPI 개발에서는 괜찮지만, 여기서는 문제가 생겼다.

이 백엔드는 단순 웹 서버가 아니라 내부에서 ROS2 노드도 같이 돌린다.

`--reload`는 파일이 바뀌면 서버 프로세스를 부모/자식 구조로 다시 띄운다. 이 과정에서 ROS2 노드와 executor thread가 꼬였다.

겉으로 보면 ROS2 서비스는 보였다.

```text
/confirm_tools
```

하지만 실제 콜백이 정상적으로 처리되지 않거나 WebSocket 브로드캐스트가 안 되는 상태가 됐다.

### 해결

`start_backend.sh`에서 `--reload`를 제거했다.

현재는 이렇게 실행한다.

```bash
python3 -m uvicorn app.main:app --port 8000
```

즉, ROS2 브릿지를 포함한 HMI 백엔드는 reload 없는 단일 프로세스로 실행해야 한다.

## 문제 2. 프론트가 HMI 백엔드 WebSocket에 실제로 안 붙어 있었음

브라우저 화면은 떠 있었지만, 실제 확인 요청을 받는 WebSocket 연결이 없었다.

화면 주소는 다음이었다.

```text
http://localhost:5174
```

그런데 프론트 코드에서는 WebSocket 주소가 다음처럼 되어 있었다.

```text
ws://localhost:8000/ws
```

실제 확인 결과, 브라우저가 `5174` 프론트에는 연결되어 있었지만 `8000/ws`에는 붙지 않은 상태였다.

그래서 백엔드 로그에 이런 메시지가 찍혔다.

```text
연결된 HMI WebSocket이 없어 확인 요청을 보류함
```

즉, `voice_processing`은 HMI에 확인 요청을 보냈지만, 브라우저 작업 화면이 그 요청을 받을 통로가 열려 있지 않았다.

### 해결

프론트 WebSocket 주소를 명시적으로 `127.0.0.1`로 바꿨다.

```text
ws://localhost:8000/ws
-> ws://127.0.0.1:8000/ws
```

그리고 HTTP API도 같은 기준으로 맞췄다.

```text
http://127.0.0.1:8000
```

또한 `5174` 포트에서 뜬 프론트도 백엔드에 접근할 수 있도록 CORS에 추가했다.

```python
allow_origins=[
    "http://localhost:5173",
    "http://127.0.0.1:5173",
    "http://localhost:5174",
    "http://127.0.0.1:5174",
]
```

## 추가로 넣은 안정화

### WebSocket 자동 재연결

백엔드가 재시작되거나 네트워크가 순간적으로 끊기면 WebSocket이 닫힌다.

그래서 프론트에서 1초 뒤 자동 재연결하도록 했다.

```text
WebSocket close
-> 1초 후 reconnect
```

### pending 요청 조회 API

WebSocket 메시지를 순간적으로 놓칠 수도 있으므로, 백엔드에 현재 대기 중인 확인 요청을 조회하는 API를 추가했다.

```text
GET /api/voice/pending
```

프론트는 1초마다 이 API를 확인해서 아직 pending 요청이 있으면 화면에 표시한다.

이건 WebSocket을 보조하는 안전장치다.

## 실행 순서

### 1. HMI 백엔드 실행

```bash
/home/rokey/ws_cobot2_pjt-web-hmi/backend/start_backend.sh
```

주의: `uvicorn --reload`로 직접 실행하지 않는다.

### 2. HMI 프론트 실행

```bash
cd /home/rokey/ws_cobot2_pjt-web-hmi/frontend
npm run dev
```

브라우저에서 보통 다음 중 하나로 접속한다.

```text
http://localhost:5173
http://localhost:5174
```

### 3. voice_processing 실행

```bash
/home/rokey/Downloads/ws_cobot2_pjt-voice_process/start_get_keyword.sh
```

### 4. 서비스 확인

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/Downloads/ws_cobot2_pjt-voice_process/install/setup.bash
export ROS_DOMAIN_ID=67
ros2 service list | grep -E "confirm_tools|get_keyword"
```

정상이라면 다음이 보여야 한다.

```text
/confirm_tools
/get_keyword
```

### 5. 수동 테스트

HMI 작업 화면을 열어둔 상태에서:

```bash
ros2 service call /confirm_tools od_msg/srv/ConfirmTools "{tools: [hammer], targets: [user]}"
```

화면에 다음이 떠야 한다.

```text
확인 필요
hammer -> user
```

## 다음 목표: HMI 버튼 대신 음성으로 확인하기

현재는 HMI 작업 화면에서 사람이 직접 `확인` 또는 `아니오` 버튼을 눌러야 한다.

다음 목표는 이 확인 과정도 음성으로 처리하는 것이다.

예:

```text
HMI/시스템: "hammer와 screwdriver를 user에게 가져다주는 것이 맞나요?"
사용자: "응 맞아"
-> confirmed=True
```

또는:

```text
사용자: "아니야"
-> confirmed=False
-> 다시 명령 듣기
```

## 음성 확인 기능 설계안

### 방식 A. `get_keyword.py` 안에서 확인 음성까지 처리

흐름:

```text
명령 음성 입력
-> 도구/목적지 추출
-> HMI에 표시
-> 동시에 "맞으면 맞아, 아니면 아니야라고 말해주세요" 안내
-> STT로 확인 답변 인식
-> confirmed=True/False
```

장점:

- 구현이 빠르다.
- 기존 STT 클래스를 그대로 재사용할 수 있다.
- HMI는 표시만 담당하고, 음성 로직은 voice_processing에 모인다.

단점:

- HMI 버튼 확인과 음성 확인이 동시에 존재하면 우선순위 처리가 필요하다.
- 사용자가 HMI에서 누르는 것과 음성으로 말하는 것이 충돌할 수 있다.

추천 우선순위:

```text
1. HMI 버튼이 먼저 들어오면 버튼 응답 사용
2. 음성 응답이 먼저 들어오면 음성 응답 사용
3. 둘 다 없으면 timeout
```

### 방식 B. 별도 `voice_confirm` ROS2 서비스 만들기

확인 전용 음성 노드를 따로 만든다.

```text
/confirm_tools
-> HMI 표시
-> /listen_confirm_voice 호출
-> "맞아"/"아니야" 인식
-> confirmed 반환
```

장점:

- 역할이 깔끔하게 나뉜다.
- 나중에 HMI, 음성, 물리 버튼 등 여러 확인 방식을 붙이기 쉽다.

단점:

- 서비스/노드가 늘어나서 구조가 복잡해진다.
- 지금 단계에서는 구현량이 많다.

### 추천

지금 단계에서는 **방식 A**를 추천한다.

이유:

- 이미 `get_keyword.py`가 음성을 듣고 STT를 수행하고 있다.
- 확인 응답은 `"응"`, `"맞아"`, `"아니야"`처럼 짧은 문장이라 LLM까지 가지 않아도 규칙 기반으로 처리 가능하다.
- 먼저 동작하는 데모를 만들기 좋다.

## 음성 확인 인식 규칙 예시

확인으로 볼 표현:

```text
응
맞아
네
그래
오케이
맞습니다
그거 맞아
응 맞아
yes
ok
```

거절로 볼 표현:

```text
아니
아니야
틀렸어
다시
그거 아니야
잘못됐어
no
cancel
```

애매한 표현:

```text
음
잠깐
뭐라고
다시 말해줘
```

애매하면 confirmed를 바로 정하지 말고 한 번 더 물어보는 것이 좋다.

```text
"확인인지 취소인지 다시 말해주세요."
```

## 추천 구현 순서

1. `get_keyword.py`에 `listen_confirmation()` 함수 추가

STT를 한 번 더 호출해서 짧은 확인 문장을 받는다.

2. `parse_confirmation(text)` 함수 추가

텍스트에 확인/거절 키워드가 포함되어 있는지 규칙 기반으로 판단한다.

예:

```python
YES_WORDS = ["응", "맞아", "네", "오케이", "yes", "ok"]
NO_WORDS = ["아니", "아니야", "틀렸어", "다시", "no", "cancel"]
```

3. HMI 확인 요청과 음성 확인을 동시에 또는 순차적으로 처리

가장 쉬운 1차 구현:

```text
HMI에 확인 요청 표시
-> 바로 확인 음성 듣기
-> 음성 결과를 confirmed로 사용
```

조금 더 좋은 2차 구현:

```text
HMI 버튼 응답과 음성 응답 중 먼저 들어온 것을 사용
```

4. HMI 화면에 음성 확인 대기 상태 표시

예:

```text
확인 필요
hammer -> user

음성으로 "맞아" 또는 "아니야"라고 말해도 됩니다.
```

5. 실패 처리

확인 음성을 못 알아들으면:

```text
다시 말해주세요
```

3번 정도 실패하면 HMI 버튼 확인으로 fallback한다.

## 추가로 개선하면 좋은 기능

### 1. 연결 상태를 더 자세히 표시

현재는 `연결됨/미연결` 정도만 표시한다.

더 좋게 만들려면:

```text
ROS2 브릿지: 연결됨
WebSocket: 연결됨
voice_processing: 대기 중
마지막 요청 시간: 18:52:10
```

이렇게 나누면 문제가 생겼을 때 어디가 끊겼는지 바로 보인다.

### 2. 현재 작업 로그 추가

작업 화면 아래에 이벤트 로그를 보여주면 디버깅이 훨씬 쉽다.

예:

```text
18:51:02 음성 명령 수신
18:51:07 STT 결과: 해머 가져다줘
18:51:08 도구 추출: hammer -> user
18:51:08 HMI 확인 요청 전송
18:51:12 사용자 확인
```

### 3. HMI에서 음성 명령 원문 표시

현재는 도구와 목적지만 보여준다.

추가로 STT 원문도 보여주면 사용자가 확인하기 좋다.

```text
인식된 문장:
"해머랑 드라이버 가져다줘"

추출 결과:
hammer -> user
screwdriver -> user
```

### 4. 도구/목적지 수정 기능

사용자가 "아니오"만 누르는 대신 HMI에서 직접 수정할 수 있게 한다.

예:

```text
hammer -> user
screwdriver -> pos1
```

이렇게 수정 후 확인하면 다시 음성을 말하지 않아도 된다.

### 5. timeout 시 화면 상태 표시

지금은 60초 동안 응답이 없으면 false로 끝난다.

화면에는 다음처럼 보여주는 것이 좋다.

```text
확인 요청 시간이 초과되었습니다.
다시 명령해주세요.
```

### 6. product 매핑을 HMI에서 관리

현재 `product1`, `product2`, `product3`에 필요한 도구 매핑이 코드 안에 있다.

나중에는 HMI 재고/작업 관리 화면에서 제품별 필요 도구를 등록하고, `voice_processing`이 그 데이터를 참고하면 더 실용적이다.

예:

```text
product1:
  - hammer
  - screwdriver

product2:
  - wrench
```

### 7. 실행 스크립트 통합

현재는 백엔드, 프론트, voice_processing을 따로 실행한다.

나중에는 다음처럼 한 번에 상태를 확인하고 띄우는 스크립트가 있으면 좋다.

```bash
./start_voice_hmi_stack.sh
```

스크립트가 확인할 것:

```text
ROS_DOMAIN_ID=67
/confirm_tools 서비스 존재 여부
/get_keyword 서비스 존재 여부
HMI backend 8000 포트
HMI frontend 5173/5174 포트
```

## 현재 구조의 핵심 한 줄 요약

```text
voice_processing은 "무엇을 할지" 알아내고,
HMI는 "이게 맞는지" 사람에게 확인받고,
confirm_tools 서비스가 그 둘 사이의 다리 역할을 한다.
```

