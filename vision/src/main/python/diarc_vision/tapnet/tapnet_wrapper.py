import logging
import time
import cv2
import jax
import jax.numpy as jnp
import numpy as np
from tapnet.models import tapir_model
from tapnet.utils import model_utils


def load_checkpoint(checkpoint_path):
    ckpt_state = np.load(checkpoint_path, allow_pickle=True).item()
    return ckpt_state["params"], ckpt_state["state"]

class Tapnet:
    def __init__(self, device: str = "cpu"):
        self.device = device
        self.is_initialized = False
        self.NUM_POINTS = 5
        self.have_point = [False] * self.NUM_POINTS
        self.visibles = [False] * self.NUM_POINTS
        self.tracks = np.zeros((self.NUM_POINTS, 2))
        self.query_points = None
        self.query_features = None
        self.causal_state = None
        self.next_query_idx = 0

        # Load checkpoint and initialize
        params, state = load_checkpoint(
            "/home/evan/code/tapnet/checkpoints/causal_tapir_checkpoint.npy"
        )

        self.tapir = tapir_model.ParameterizedTAPIR(
            params=params,
            state=state,
            tapir_kwargs=dict(
                use_causal_conv=True, bilinear_interp_with_depthwise_conv=False
            ),
        )

        print("Please note that if the framerate is low (<~12 fps), TAPIR performance")
        print("may degrade and you may need a more powerful GPU.")

        print("Creating model...")
        self.online_init_apply = jax.jit(self.online_model_init)
        self.online_predict_apply = jax.jit(self.online_model_predict)

    def one_time_init(self, frame):
        if self.is_initialized:
            return

        print("Compiling jax functions (this may take a while...)")
        # --------------------
        # Call one time to compile
        self.query_points = jnp.zeros([self.NUM_POINTS, 3], dtype=jnp.float32)
        _ = self.online_init_apply(
            frames=model_utils.preprocess_frames(frame[None, None]),
            points=self.query_points[None, 0:1],
        )
        jax.block_until_ready(self.query_features)
        self.query_features = self.online_init_apply(
            frames=model_utils.preprocess_frames(frame[None, None]),
            points=self.query_points[None, :],
        )

        self.causal_state = self.tapir.construct_initial_causal_state(
            self.NUM_POINTS, len(self.query_features.resolutions) - 1
        )

        self.prediction, causal_state = self.online_predict_apply(
            frames=model_utils.preprocess_frames(frame[None, None]),
            features=self.query_features,
            causal_context=self.causal_state,
        )

        jax.block_until_ready(self.prediction["tracks"])

        self.is_initialized = True

    def online_model_init(self, frames, points):
        feature_grids = self.tapir.get_feature_grids(frames, is_training=False)
        features = self.tapir.get_query_features(
            frames,
            is_training=False,
            query_points=points,
            feature_grids=feature_grids,
        )
        return features

    def online_model_predict(self, frames, features, causal_context):
        """Compute point tracks and occlusions given frames and query points."""
        feature_grids = self.tapir.get_feature_grids(frames, is_training=False)
        trajectories = self.tapir.estimate_trajectories(
            frames.shape[-3:-1],
            is_training=False,
            feature_grids=feature_grids,
            query_features=features,
            query_points_in_video=None,
            query_chunk_size=64,
            causal_context=causal_context,
            get_causal_context=True,
        )
        causal_context = trajectories["causal_context"]
        del trajectories["causal_context"]
        return {k: v[-1] for k, v in trajectories.items()}, causal_context

    def add_track_point(self, pos, frame) -> int:
        self.next_query_idx = self.get_next_track_id()
        if self.next_query_idx is None:
            logging.warning("Can't add new track until an existing track is removed.")
            return None

        tapnet_pos = (pos[1], frame.shape[1] - pos[0]) # (y, width-x)
        query_points = jnp.array((0,) + tapnet_pos, dtype=jnp.float32)

        init_query_features = self.online_init_apply(
            frames=model_utils.preprocess_frames(frame[None, None]),
            points=query_points[None, None],
        )
        self.query_features, self.causal_state = self.tapir.update_query_features(
            query_features=self.query_features,
            new_query_features=init_query_features,
            idx_to_update=np.array([self.next_query_idx]),
            causal_state=self.causal_state,
        )
        self.have_point[self.next_query_idx] = True
        return self.next_query_idx

    def remove_track_point(self, track_idx: int):
        self.have_point[track_idx] = False

    def get_next_track_id(self):
        next_idx = self.have_point.index(False) if False in self.have_point else None
        if next_idx is None:
            logging.warning("All valid track IDs are being used. Please remove a track first.")
        return next_idx

    def update_tracks(self, frame):
        prediction, causal_state = self.online_predict_apply(
            frames=model_utils.preprocess_frames(frame[None, None]),
            features=self.query_features,
            causal_context=self.causal_state,
        )
        track = prediction["tracks"][0, :, 0]
        occlusion = prediction["occlusion"][0, :, 0]
        expected_dist = prediction["expected_dist"][0, :, 0]
        self.visibles = model_utils.postprocess_occlusions(occlusion, expected_dist)
        self.tracks = np.round(track)

    def get_track_position(self, track_id):
        if self.visibles[track_id] and self.have_point[track_id]:
            return int(self.tracks[track_id, 0]), int(self.tracks[track_id, 1])
        else:
            return None

if __name__ == '__main__':

    # start capture
    vc = cv2.VideoCapture(0)
    if not vc.isOpened():
        raise ValueError("Unable to open camera.")

    def get_frame(video_capture):
        r_val, image = video_capture.read()

        image = cv2.resize(image, (320, 240))
        # trunc = np.abs(image.shape[1] - image.shape[0]) // 2
        # if image.shape[1] > image.shape[0]:
        #   image = image[:, trunc:-trunc]
        # elif image.shape[1] < image.shape[0]:
        #   image = image[trunc:-trunc]
        return r_val, image

    rval, frame = get_frame(vc)
    print(f"frame size: {frame.shape}")

    # create Tapnet tracker (and do one-time init)
    tapnet = Tapnet()
    tapnet.one_time_init(frame)


    def mouse_click(event, x, y, flags, param):
        del flags, param
        global clicked_pos, new_clicked_pos, last_click_time

        # event fires multiple times per click sometimes??
        if (time.time() - last_click_time) < 0.5:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_pos = (x, y)
            new_clicked_pos = True
            last_click_time = time.time()


    cv2.namedWindow("Point Tracking")
    cv2.setMouseCallback("Point Tracking", mouse_click)

    print("Press ESC to exit.")

    new_clicked_pos = False
    last_click_time = time.time()
    is_tracking_pts = False
    step_counter = 0

    while rval:
        rval, frame = get_frame(vc)

        if new_clicked_pos:
            new_clicked_pos = False
            tapnet.add_track_point(clicked_pos, frame)

        tapnet.update_tracks(frame)

        for i in range(tapnet.NUM_POINTS):
            if tapnet.visibles[i] and tapnet.have_point[i]:
                is_tracking_pts = True
                cv2.circle(
                    frame, (int(tapnet.tracks[i, 0]), int(tapnet.tracks[i, 1])), 5, (255, 0, 0), -1
                )

        cv2.imshow("Point Tracking", frame[:, ::-1])
        key = cv2.waitKey(1)

        if key == 27:  # exit on ESC
            break

        if is_tracking_pts:
            step_counter += 1
            if time.time() - t > 5:
                print(f"{step_counter / (time.time() - t)} frames per second")
                t = time.time()
                step_counter = 0
        else:
            t = time.time()

    cv2.destroyWindow("Point Tracking")
    vc.release()
